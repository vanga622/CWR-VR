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

struct StereoFrame
{
    EyeView eyes[2];
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
