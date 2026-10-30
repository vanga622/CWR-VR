#pragma once

// CWR-VR: OpenXR session for the Poseidon engine.
//
// One OpenXR session on the game's own OpenGL context (XR_KHR_opengl_enable).
// Two ways to present a frame:
//   - flat (M1): the finished window frame on a quad floating in front of
//     the player — used for loading screens and any frame without stereo;
//   - stereo (M2): one image per eye in a projection layer, plus the HUD on
//     a head-locked quad.
// The library makes no GL calls itself: it hands the graphics backend
// swapchain textures, the backend fills them, and the session submits them.
//
// Everything is logged to vr.log next to the executable.  Any failure (no
// runtime, no headset, missing extension) leaves the session inactive and the
// game keeps running flat.

namespace PoseidonVR
{

// Tunables, read from vr.ini next to the executable (written with defaults
// when missing).
struct VRConfig
{
    float eyeScale = 0.6f;       // eye image size relative to the runtime's recommendation
    float hudDistance = 1.5f;    // metres in front of the eyes
    float hudHalfWidthTan = 0.6f; // horizontal half-size of the HUD as tan(angle)
    float screenDistance = 2.5f; // flat virtual screen, metres
    float screenWidth = 3.2f;    // flat virtual screen, metres (16:9)
    float smoothTurn = 0.0f;     // 0 = snap turn, 1 = smooth turn
    float snapAngle = 30.0f;     // degrees per snap
    float smoothSpeed = 120.0f;  // degrees per second at full stick
    float locomotion = 2.0f;     // left stick "forward": 0 = body, 1 = head, 2 = left controller
    // Weapon placement: where the weapon model's centre sits relative to the
    // right hand, in the controller's aim frame (metres).
    float weaponForward = 0.15f;
    float weaponUp = 0.06f;
    float weaponRight = 0.0f;
    float twoHanded = 1.0f;      // 1 = left grip near the weapon steadies it with both hands
    float foreEndForward = 0.25f; // where the left hand holds the weapon, ahead of its centre
    // Zeroing: turns the shot (not the model) so hits land on the sights.
    float zeroYaw = 0.0f;        // degrees, + = hits move right
    float zeroPitch = 0.0f;      // degrees, + = hits move up
    // Shot origin shift (a constant offset at every distance), centimetres.
    float muzzleRight = 0.0f;    // + = hits move right
    float muzzleUp = 0.0f;       // + = hits move up
    // Physical stance: a real crouch lowers the view (the soldier keeps
    // standing; the game has no rifle crouch), lying down past proneDrop
    // below standing height makes him go prone.
    float physicalStance = 1.0f; // 1 = on
    float proneDrop = 0.65f;     // metres below standing height
    float eyeHeight = 0.0f;      // metres added to the soldier's eye height (taller/shorter player)
    // Sideways/forward head moves kept before the soldier's body "follows"
    // (beyond it, head and hands are pulled back over the body so the arms
    // never stretch and the view never enters the shoulders), metres.
    float leanFree = 0.05f;
    // Manual reload (M6).  Places are in metres: the pouch relative to the
    // head (body heading), the magazine well and charging handle relative to
    // the weapon centre (along its barrel/up axes).
    float manualReload = 1.0f;   // 1 = B ejects, magazine from pouch by hand, charge by hand
    float pouchForward = 0.20f;
    float pouchDown = 0.50f;
    float pouchRight = -0.05f;
    float pouchRadius = 0.30f;
    float wellForward = 0.05f;   // magazine well: ahead of the weapon centre
    float wellDown = 0.10f;      // and below it
    float wellRadius = 0.18f;
    float chargeBack = 0.20f;    // charging handle: behind the weapon centre
    float chargeUp = 0.05f;
    float chargeRadius = 0.12f;
    float chargePull = 0.06f;    // pull back this far to charge
};

// Controller state from the OpenXR actions (Touch layout), refreshed every
// frame while the session has input focus; all zero otherwise.
struct ControllerState
{
    bool active = false;
    float moveX = 0, moveY = 0; // left thumbstick, +Y = forward
    float turnX = 0, turnY = 0; // right thumbstick
    float trigger[2] = {0, 0};  // [0] left hand, [1] right hand
    float grip[2] = {0, 0};
    bool a = false, b = false, x = false, y = false, menu = false;
    bool thumbClick[2] = {false, false};
};

// Swapchain image the backend must fill with the current flat frame.
struct FlatFrameTarget
{
    unsigned int glTexture = 0; // GL_TEXTURE_2D name owned by the runtime
    int width = 0;
    int height = 0;
};

// One eye of a stereo frame, in OpenXR LOCAL space (right-handed, +Y up,
// -Z forward, metres).
struct EyeView
{
    float orientation[4] = {0, 0, 0, 1}; // quaternion x, y, z, w
    float position[3] = {0, 0, 0};
    float tanLeft = -1, tanRight = 1, tanUp = 1, tanDown = -1; // FOV edges as tangents
};

// A tracked controller pose in the same LOCAL space as the eyes.
struct HandPose
{
    bool valid = false;
    float orientation[4] = {0, 0, 0, 1};
    float position[3] = {0, 0, 0};
};

struct StereoFrame
{
    EyeView eyes[2];
    HandPose grip[2]; // [0] left, [1] right: where the hand holds the controller
    HandPose aim[2];  // pointing ray (-Z forward)
    float headHeight = -1.0f; // eyes above the real floor (STAGE space), metres; < 0 unknown
    int eyeWidth = 0;
    int eyeHeight = 0;
    unsigned int eyeTexture[2] = {0, 0}; // 0 in fake mode: nothing to submit
    unsigned int hudTexture = 0;
    int hudWidth = 0;
    int hudHeight = 0;
};

class VRSession
{
public:
    static VRSession& Instance();

    // Create the instance, session and swapchains.  The game's GL context
    // must be current on the calling thread.  Returns false (and logs why)
    // when VR is unavailable.
    bool Init();
    // Fake headset (--vr-fake): no runtime, fixed scripted eye poses, so
    // stereo can be checked from screenshots.
    bool InitFake();
    void Shutdown();

    bool IsActive() const;
    bool IsFake() const;
    const VRConfig& Config() const;
    // Live tunables: the dev panel's sliders edit these directly.
    VRConfig& MutableConfig();
    // Re-reads vr.ini when the file changed on disk (checked at most once a
    // second).  Returns true when new values were loaded.  eye_scale takes
    // effect on the next start (swapchains are sized once).
    bool ReloadConfigIfChanged();
    void SaveConfig();

    // Latest controller state (synced with each Begin*Frame).
    const ControllerState& Controllers() const;
    // Latest hand poses, located with each Begin*Frame (flat frames too).
    const HandPose& Aim(int hand) const;
    const HandPose& Grip(int hand) const;
    // Short vibration on a controller (0 left, 1 right); amplitude 0..1.
    void Haptic(int hand, float amplitude, float seconds);

    // One line in vr.log, for VR events decided outside this library
    // (recentre, heading re-sync).
    void LogLine(const char* text);

    // Flat frame, once per presented frame before SwapWindow: polls runtime
    // events and, while the session runs, waits for/begins an XR frame.
    // Returns true when `out` holds an acquired image the caller must fill
    // and hand back with EndFlatFrame.  False = nothing to draw (any begun XR
    // frame is already ended).
    bool BeginFlatFrame(FlatFrameTarget& out);
    void EndFlatFrame();

    // Stereo frame, before the scene is drawn: polls events, waits for and
    // begins an XR frame, locates both eyes and acquires the eye and HUD
    // images.  False = draw flat this frame (any begun XR frame is ended).
    bool BeginStereoFrame(StereoFrame& out);
    // Releases the images and submits the projection layer and the HUD quad
    // (head-locked, size from the HUD's horizontal/vertical half tangents).
    void EndStereoFrame(float hudHalfTanX, float hudHalfTanY);

private:
    VRSession() = default;
    VRSession(const VRSession&) = delete;
    VRSession& operator=(const VRSession&) = delete;
};

} // namespace PoseidonVR
