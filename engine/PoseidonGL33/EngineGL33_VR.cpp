// VR: OpenXR for the GL33 backend (CWR-VR).
//
// PoseidonVR owns the OpenXR session and hands out swapchain textures; this
// file does the GL side.
//
// Stereo frame (M2), driven by World through the Engine VR hooks:
//   VRScenePassCount   begin the XR frame, locate the eyes
//   VRBeginScenePass   per eye: camera = game camera (yaw only) + head pose,
//                      asymmetric frustum, render target = that eye's FBO
//   VRBeginHUDPass     window frame target cleared to transparent, camera =
//                      centre eye with the HUD quad's FOV
//   SubmitVRFrame      eyes -> eye swapchains, window frame -> HUD swapchain
// Frames that never reach VRScenePassCount (loading, splash) go to the flat
// virtual screen instead (M1).
//
// Heading (M3): VR owns the view heading (s_playerYaw), turned by the right
// stick, and the game aims where the head looks (VRAimDirection), so the
// soldier turns after the player instead of the other way round.  The view
// re-syncs to the game camera when they stay far apart (cutscene, respawn,
// vehicle).  Controllers reach the game as a virtual gamepad (VRGamepad).

#include <PoseidonGL33/EngineGL33.hpp>
#include <Poseidon/Dev/Debug/DebugOverlay.hpp>
#include <Poseidon/Foundation/Platform/AppConfig.hpp>
#include <Poseidon/Graphics/Core/GLClear.hpp>
#include <Poseidon/Graphics/Core/GLPipelineState.hpp>
#include <Poseidon/Input/InputSubsystem.hpp>
#include <Poseidon/World/Scene/Camera/Camera.hpp>
#include <PoseidonVR/VRGamepad.hpp>
#include <PoseidonVR/VRSession.hpp>

#include <SDL3/SDL.h>

#include <cstdlib>

extern void SDLInput_GamepadDebug(char* buf, int size); // InputProcessingSdl.cpp
#include <cfloat>
#include <chrono>
#include <cmath>
#include <glad/gl.h>
// The engine's DebugLog macro collides with ImGui::DebugLog; this TU does not
// use the macro (same as DebugOverlay.cpp).
#ifdef DebugLog
#undef DebugLog
#endif
#include <imgui.h>

namespace
{

// Offscreen target one eye renders into (MSAA like the window target), plus
// the single-sample image it resolves to before going to the swapchain.
struct EyeTarget
{
    GLuint fbo = 0;
    GLuint colorRb = 0;
    GLuint depthRb = 0;
    GLuint resolveFbo = 0;
    GLuint resolveRb = 0;
    int width = 0;
    int height = 0;
    int samples = 0;
};

EyeTarget s_eyes[2];
PoseidonVR::StereoFrame s_frame;
bool s_mirror = false;

// Game camera of the current frame, before head tracking.
Matrix4 s_gameCamera;
Matrix3 s_bodyYaw;
float s_near = 0.1f;
float s_far = 1000.0f;
float s_hudTanX = 0.6f;
float s_hudTanY = 0.34f;

// Heading and controllers (M3).
PoseidonVR::VRGamepad s_gamepad;
bool s_yawValid = false;     // false until the first stereo frame
float s_playerYaw = 0.0f;    // world heading of the tracking space's forward, radians
Vector3 s_headOrigin(0, 0, 0); // tracking-space head position placed at the game camera
float s_moveYaw = 0.0f;      // view heading minus body heading (head-relative walking)
bool s_snapArmed = true;

// The player's body this frame (Engine::VRSetPlayerBody), and what the last
// stereo frame saw, to tell a cut (respawn, vehicle, cutscene, teleport)
// from ordinary turning.
const void* s_bodyId = nullptr;
float s_bodyHeading = 0.0f;
bool s_bodySoldier = false;
float s_bodyGroundY = 0.0f;
// The standing soldier's eyes above the ground, learnt where he stands on
// open terrain (camera minus terrain height, when plausible).
float s_soldierEyeHeight = 1.75f;
bool s_soldierEyeKnown = false;
const void* s_lastBodyId = nullptr;
Vector3 s_lastCameraPos(0, 0, 0);
bool s_recentreHeld = false;
int s_recentreCount = 0; // bumped by every recentre (standing height restarts)
bool s_aimValid = false;
Vector3 s_aimDir(0, 0, 1);
std::chrono::steady_clock::time_point s_lastTurnTime;

constexpr float kDegToRad = 3.14159265f / 180.0f;

// Heading (radians, clockwise from +Z) of a frame, from its aside vector so
// it stays defined when looking straight up or down.
float HeadingOf(const Matrix3& m)
{
    Vector3Val aside = m.DirectionAside();
    return std::atan2(-aside.Z(), aside.X());
}

Matrix3 YawMatrix(float heading)
{
    Matrix3 m;
    m.SetUpAndDirection(VUp, Vector3(std::sin(heading), 0, std::cos(heading)));
    return m;
}

float WrapAngle(float a)
{
    while (a > 3.14159265f)
        a -= 2 * 3.14159265f;
    while (a < -3.14159265f)
        a += 2 * 3.14159265f;
    return a;
}

// Right stick turning, recentre (right stick click) and re-sync with the
// game camera.  Runs once per stereo frame, before the eyes are placed.
//
// Re-sync only on a cut: another body (respawn, unit switch, vehicle in or
// out, a cutscene camera) or the camera jumping in one frame (teleport).
// Ordinary turning never re-syncs: the soldier may lag behind a fast real
// turn, but the view stays the player's.
void UpdateHeading(float headYaw, const Vector3& headPos, float gameYaw, const Vector3& cameraPos, bool leftValid,
                   float leftYaw)
{
    const auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - s_lastTurnTime).count();
    s_lastTurnTime = now;
    if (dt > 0.1f)
        dt = 0.1f;

    auto& vr = PoseidonVR::VRSession::Instance();
    const PoseidonVR::VRConfig& cfg = vr.Config();
    const PoseidonVR::ControllerState& c = vr.Controllers();

    const bool recentre = c.thumbClick[1] && !s_recentreHeld;
    s_recentreHeld = c.thumbClick[1];
    if (!s_yawValid || recentre)
    {
        // Current view straight down the game camera, head at its position.
        s_playerYaw = gameYaw - headYaw;
        s_headOrigin = headPos;
        s_yawValid = true;
        ++s_recentreCount;
        LOG_INFO(Graphics, "VR: recentred");
        char line[128];
        std::snprintf(line, sizeof(line), "Recentred (%s), eyes %.2f m above the floor",
                      recentre ? "right stick click" : "first stereo frame", s_frame.headHeight);
        vr.LogLine(line);
    }

    if (c.active)
    {
        const float x = c.turnX;
        if (cfg.smoothTurn >= 0.5f)
        {
            if (std::fabs(x) > 0.15f)
                s_playerYaw += x * cfg.smoothSpeed * kDegToRad * dt;
        }
        else if (s_snapArmed && std::fabs(x) > 0.7f)
        {
            s_playerYaw += (x > 0 ? 1.0f : -1.0f) * cfg.snapAngle * kDegToRad;
            s_snapArmed = false;
        }
        else if (std::fabs(x) < 0.3f)
        {
            s_snapArmed = true;
        }
    }
    s_playerYaw = WrapAngle(s_playerYaw);

    // A frame without a reported body (menu, pause, a moment of free camera)
    // is not a new body; only a different one is.
    const bool bodyChanged = s_bodyId && s_lastBodyId && s_bodyId != s_lastBodyId;
    const float jump = (cameraPos - s_lastCameraPos).Size();
    if (bodyChanged || (s_bodyId && jump > 10.0f)) // 10 m in one frame: faster than any vehicle
    {
        const float diff = WrapAngle(gameYaw - (s_playerYaw + headYaw));
        s_playerYaw = WrapAngle(gameYaw - headYaw);
        char line[128];
        std::snprintf(line, sizeof(line), "View re-synced to the game camera (%s, %.0f deg)",
                      bodyChanged ? "new body" : "camera jump", diff / kDegToRad);
        LOG_INFO(Graphics, "VR: {}", line);
        vr.LogLine(line);
    }
    if (s_bodyId)
    {
        s_lastBodyId = s_bodyId;
        s_lastCameraPos = cameraPos;
    }

    // The game moves the soldier relative to its body; turn the stick so
    // "forward" is the chosen direction.  The left controller (default)
    // lets the player look around while walking straight; the head is
    // "walk where I look"; the body follows the weapon.  Yaws of the head
    // and the left controller are in tracking space, so add the player's
    // heading.
    const float bodyYaw = s_bodyId ? s_bodyHeading : gameYaw;
    const int mode = static_cast<int>(cfg.locomotion + 0.5f);
    float forwardYaw = bodyYaw;
    if (mode == 2 && leftValid)
        forwardYaw = s_playerYaw + leftYaw;
    else if (mode >= 1)
        forwardYaw = s_playerYaw + headYaw;
    s_moveYaw = WrapAngle(forwardYaw - bodyYaw);
}

// Weapon in the right hand (M4).
bool s_weaponValid = false;
bool s_twoHanded = false;
bool s_foreGrip = false; // left hand holds the fore-end (until the grip lets go)
Matrix4 s_weaponWorld;

// Physical stance (M5): head drop below the player's standing height, and
// the vertical shift that maps the real head height onto the soldier.
bool s_headDropValid = false;
float s_headDrop = 0.0f;
float s_standY = 0.0f;  // highest head height since recentre = standing
bool s_gameProne = false; // the soldier lies (Engine::VRSetProne)
float s_verticalShift = 0.0f;
float s_horizontalShift[2] = {0.0f, 0.0f}; // x, z: keeps the body under the head

// Menu pointer (M5): right controller ray -> UI cursor, trigger -> click.
bool s_pointerDown = false;
bool s_uiInput = true;    // UI (pointer) rather than gameplay this frame
bool s_dialogOpen = false; // World: a dialog is up (Engine::VRSetDialogOpen)

// Manual reload (M6): the soldier's weapon state (VRSetMagazineState) and
// the gesture in progress.
bool s_magStateValid = false;
bool s_magLoaded = true;
int s_magSpares = 0;
bool s_chambered = true;
bool s_magInHand = false;   // left hand holds a magazine (from the pouch or the well)
bool s_insertArmed = true;  // the hand has been away from the well since it took the magazine
bool s_charging = false;    // left hand holds the charging handle
float s_chargeStart = 0.0f; // where along the barrel it was grabbed
float s_boltPull = 0.0f;    // how far the handle is back, 0..1
bool s_boltBack = false;    // it reached the end: going forward chambers a round
// The charging handle of the weapon drawn, in the weapon's frame (metres
// ahead, up, right of its centre); not valid for weapons with no moving one.
bool s_handleValid = false;
Vector3 s_handleLocal = VZero;
float s_handleTravel = 0.0f;
bool s_ejectHeld = false;
bool s_leftGripHeld = false;
bool s_pouchSetHeld = false;
int s_weaponKey = -1;       // the soldier's weapon (VRSetMagazineState)
// Events for the soldier, oldest first, each with where the magazine was.
struct ReloadEvent
{
    int kind = Engine::VRReloadNone;
    Matrix4 pose = MIdentity;
    Vector3 velocity = VZero;
};
ReloadEvent s_events[4];
int s_eventCount = 0;
ReloadEvent s_lastEvent;
// Left hand velocity (world, m/s), for a magazine let go in motion.
Vector3 s_leftPrev = VZero;
Vector3 s_leftVelocity = VZero;
std::chrono::steady_clock::time_point s_leftPrevTime;

void PushReloadEvent(int kind, const Matrix4& pose = MIdentity, const Vector3& velocity = VZero)
{
    if (s_eventCount == 4)
    {
        for (int i = 1; i < 4; ++i)
            s_events[i - 1] = s_events[i];
        --s_eventCount;
    }
    s_events[s_eventCount++] = ReloadEvent{kind, pose, velocity};
}

// Where the left hand takes the charging handle, for a weapon at this pose.
Vector3 ChargeHandle(const Matrix3& orient, Vector3Par centre)
{
    const PoseidonVR::VRConfig& cfg = PoseidonVR::VRSession::Instance().Config();
    if (s_handleValid)
        return centre + orient.Direction() * s_handleLocal.X() + orient.DirectionUp() * s_handleLocal.Y() +
               orient.DirectionAside() * s_handleLocal.Z();
    return centre - orient.Direction() * cfg.chargeBack + orient.DirectionUp() * cfg.chargeUp;
}

// The magazine models lie flat (long axis +Z, thin axis +Y).  In the hand
// the long axis stands up, tipped forward by mag_pitch, thin side across.
Matrix4 MagazineInHand(const Matrix4& hand, bool withScale)
{
    const PoseidonVR::VRConfig& cfg = PoseidonVR::VRSession::Instance().Config();
    const float pitch = cfg.magPitch * kDegToRad;
    Matrix3 orient;
    orient.SetDirectionAndUp(Vector3(0.0f, std::cos(pitch), std::sin(pitch)), Vector3(1.0f, 0.0f, 0.0f));
    Matrix4 local = MIdentity;
    local.SetOrientation(withScale ? orient * Matrix3(MScale, cfg.magScale) : orient);
    local.SetPosition(Vector3(0.0f, -cfg.magDown, cfg.magForward));
    return hand * local;
}

// The magazine in the well: top up into the weapon, thin side across.
Matrix4 MagazineInWell(const Matrix4& weapon)
{
    const PoseidonVR::VRConfig& cfg = PoseidonVR::VRSession::Instance().Config();
    Matrix4 m = MIdentity;
    Matrix3 orient;
    orient.SetDirectionAndUp(weapon.DirectionUp(), weapon.DirectionAside());
    m.SetOrientation(orient);
    m.SetPosition(weapon.Position() + weapon.Direction() * cfg.wellForward - weapon.DirectionUp() * cfg.wellDown);
    return m;
}
// Short-lived confirmation on the HUD ("Magazine out", "Charged", ...).
const char* s_flashText = nullptr;
std::chrono::steady_clock::time_point s_flashUntil;

void Flash(const char* text)
{
    s_flashText = text;
    s_flashUntil = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
}


void DestroyEyeTarget(EyeTarget& t)
{
    if (t.fbo)
        glDeleteFramebuffers(1, &t.fbo);
    if (t.resolveFbo)
        glDeleteFramebuffers(1, &t.resolveFbo);
    GLuint rbs[] = {t.colorRb, t.depthRb, t.resolveRb};
    for (GLuint rb : rbs)
        if (rb)
            glDeleteRenderbuffers(1, &rb);
    t = EyeTarget{};
}

bool EnsureEyeTarget(EyeTarget& t, int width, int height, int samples)
{
    if (t.fbo && t.width == width && t.height == height && t.samples == samples)
        return true;
    DestroyEyeTarget(t);

    glGenRenderbuffers(1, &t.colorRb);
    glBindRenderbuffer(GL_RENDERBUFFER, t.colorRb);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, width, height);
    glGenRenderbuffers(1, &t.depthRb);
    glBindRenderbuffer(GL_RENDERBUFFER, t.depthRb);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH24_STENCIL8, width, height);
    glGenFramebuffers(1, &t.fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, t.fbo);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, t.colorRb);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, t.depthRb);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    glGenRenderbuffers(1, &t.resolveRb);
    glBindRenderbuffer(GL_RENDERBUFFER, t.resolveRb);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height);
    glGenFramebuffers(1, &t.resolveFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, t.resolveFbo);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, t.resolveRb);
    const GLenum resolveStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (status != GL_FRAMEBUFFER_COMPLETE || resolveStatus != GL_FRAMEBUFFER_COMPLETE)
    {
        LOG_ERROR(Graphics, "VR: eye target {}x{} incomplete (0x{:x}/0x{:x})", width, height, status, resolveStatus);
        DestroyEyeTarget(t);
        return false;
    }
    t.width = width;
    t.height = height;
    t.samples = samples;
    LOG_INFO(Graphics, "VR: eye target {}x{} ({}x MSAA)", width, height, samples);
    return true;
}

// OpenXR: right-handed, +Y up, -Z forward.  Engine: left-handed, +Y up,
// +Z forward (Direction), +X right (DirectionAside).  Mirror Z.
Vector3 ToEnginePosition(const float p[3])
{
    return Vector3(p[0], p[1], -p[2]);
}

Matrix3 ToEngineOrientation(const float q[4])
{
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    // v' = v + 2w(u x v) + 2u x (u x v), u = (x, y, z)
    auto rotate = [&](float vx, float vy, float vz, float out[3])
    {
        const float cx = y * vz - z * vy, cy = z * vx - x * vz, cz = x * vy - y * vx;
        const float ccx = y * cz - z * cy, ccy = z * cx - x * cz, ccz = x * cy - y * cx;
        out[0] = vx + 2 * (w * cx + ccx);
        out[1] = vy + 2 * (w * cy + ccy);
        out[2] = vz + 2 * (w * cz + ccz);
    };
    float up[3], forward[3];
    rotate(0, 1, 0, up);
    rotate(0, 0, -1, forward);
    Matrix3 m;
    m.SetDirectionAndUp(Vector3(forward[0], forward[1], -forward[2]), Vector3(up[0], up[1], -up[2]));
    return m;
}

void PlaceCamera(Camera& camera, const Matrix3& headOrientation, const Vector3& headPosition)
{
    Matrix4 t = s_gameCamera;
    t.SetOrientation(s_bodyYaw * headOrientation);
    t.SetPosition(s_gameCamera.Position() + s_bodyYaw * headPosition);
    camera.SetTransform(t);
}

// Tracking-space position -> offset from the game camera (before heading).
Vector3 TrackedOffset(const float p[3])
{
    Vector3 offset = ToEnginePosition(p) - s_headOrigin;
    offset[0] += s_horizontalShift[0];
    offset[1] += s_verticalShift;
    offset[2] += s_horizontalShift[1];
    return offset;
}

// Tracking-space position -> world, the same mapping the eyes use.
Vector3 TrackedToWorld(const float p[3])
{
    return s_gameCamera.Position() + s_bodyYaw * TrackedOffset(p);
}

// Head height for the physical stance.  Standing height is the highest the
// head has been since recentre, so it calibrates itself the moment the
// player stands straight (and a recentre done while bent over does not
// leave him "crouched").  Standing soldier: the view follows a real crouch
// down (peeking under cover) and stays at the soldier's eyes when standing.
// Prone soldier: the view sits at his eyes; lying down for real is what put
// him there (Man::VRWantedStanceAction).
void UpdateHeadDrop(const Vector3& headPos)
{
    const PoseidonVR::VRConfig& cfg = PoseidonVR::VRSession::Instance().Config();
    static int s_seenRecentre = -1;
    if (!s_headDropValid || s_seenRecentre != s_recentreCount || headPos.Y() > s_standY)
        s_standY = headPos.Y();
    s_seenRecentre = s_recentreCount;
    const float belowStand = headPos.Y() - s_standY; // <= 0
    s_headDrop = -belowStand;
    s_headDropValid = true;

    // Learn the standing soldier's eye height on open ground (indoors the
    // camera is a floor above the terrain and the value is out of range).
    if (s_bodyId && s_bodySoldier && !s_gameProne)
    {
        const float eye = s_gameCamera.Position().Y() - s_bodyGroundY;
        if (eye > 1.3f && eye < 2.1f)
        {
            s_soldierEyeHeight = s_soldierEyeKnown ? s_soldierEyeHeight + (eye - s_soldierEyeHeight) * 0.02f : eye;
            if (!s_soldierEyeKnown)
            {
                char line[96];
                std::snprintf(line, sizeof(line), "Soldier's eyes %.2f m above the ground, yours %.2f m", eye,
                              s_frame.headHeight);
                PoseidonVR::VRSession::Instance().LogLine(line);
            }
            s_soldierEyeKnown = true;
        }
    }

    // TrackedOffset's y is headPos - origin; shift it to the height wanted
    // (relative to the game camera).
    const float fromOrigin = headPos.Y() - s_headOrigin.Y();
    float wanted = fromOrigin;
    if (cfg.physicalStance >= 0.5f)
    {
        if (s_gameProne)
            wanted = 0.0f; // the prone soldier's own eyes
        else if (s_bodyId && s_bodySoldier && s_frame.headHeight > 0.0f)
            // Standing soldier, real floor known: eyes as high above his feet
            // as the player's are above the floor, so standing feels like
            // standing and a real crouch is a crouch.  His feet are his eyes
            // minus his own (calibrated) eye height, which also holds on
            // building floors.
            wanted = s_frame.headHeight - s_soldierEyeHeight;
        else
            wanted = belowStand; // no floor: the soldier's eyes, lowered by a real crouch
    }
    s_verticalShift = wanted - fromOrigin + cfg.eyeHeight;

    // Sideways/forward: the soldier does not walk with the player's real
    // steps, so keep only lean_free of it; the rest moves head and hands
    // together back over the body.
    const float dx = headPos.X() - s_headOrigin.X();
    const float dz = headPos.Z() - s_headOrigin.Z();
    const float dist = std::sqrt(dx * dx + dz * dz);
    const float freeR = cfg.leanFree < 0.0f ? 0.0f : cfg.leanFree;
    const float pull = dist > freeR ? (dist - freeR) / dist : 0.0f;
    s_horizontalShift[0] = -dx * pull;
    s_horizontalShift[1] = -dz * pull;
}

// OpenXR-space rotation of v by quaternion q (x, y, z, w).
void RotateXr(const float q[4], const float v[3], float out[3])
{
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float cx = y * v[2] - z * v[1], cy = z * v[0] - x * v[2], cz = x * v[1] - y * v[0];
    out[0] = v[0] + 2 * (w * cx + y * cz - z * cy);
    out[1] = v[1] + 2 * (w * cy + z * cx - x * cz);
    out[2] = v[2] + 2 * (w * cz + x * cy - y * cx);
}

float Dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// A rectangle in tracking space the controller can point at: the HUD quad
// in a stereo frame, the flat virtual screen otherwise.
struct PointerSurface
{
    float centre[3];
    float right[3]; // unit axes; normal = right x up, facing the player
    float up[3];
    float normal[3];
    float halfW;
    float halfH;
};

// Ray from the right controller -> UI cursor (-1..+1, y down), or false
// when it misses the surface.
bool PointAt(const PoseidonVR::HandPose& aim, const PointerSurface& s, float& x, float& y)
{
    const float fwd[3] = {0, 0, -1};
    float rayDir[3];
    RotateXr(aim.orientation, fwd, rayDir);
    const float denom = Dot3(rayDir, s.normal);
    if (denom > -1e-4f)
        return false; // pointing away
    const float toCentre[3] = {s.centre[0] - aim.position[0], s.centre[1] - aim.position[1],
                               s.centre[2] - aim.position[2]};
    const float t = Dot3(toCentre, s.normal) / denom;
    const float hit[3] = {aim.position[0] + rayDir[0] * t - s.centre[0], aim.position[1] + rayDir[1] * t - s.centre[1],
                          aim.position[2] + rayDir[2] * t - s.centre[2]};
    x = Dot3(hit, s.right) / s.halfW;
    y = -Dot3(hit, s.up) / s.halfH;
    if (x < -1.2f || x > 1.2f || y < -1.2f || y > 1.2f)
        return false;
    x = x < -1 ? -1 : x > 1 ? 1 : x;
    y = y < -1 ? -1 : y > 1 ? 1 : y;
    return true;
}

// HUD quad as submitted: head-locked, hudDistance ahead of the eyes.
PointerSurface HudSurface()
{
    const auto& eyes = s_frame.eyes;
    const float d = PoseidonVR::VRSession::Instance().Config().hudDistance;
    const float ax[3] = {1, 0, 0}, ay[3] = {0, 1, 0}, az[3] = {0, 0, 1};
    PointerSurface s;
    RotateXr(eyes[0].orientation, ax, s.right);
    RotateXr(eyes[0].orientation, ay, s.up);
    RotateXr(eyes[0].orientation, az, s.normal);
    for (int i = 0; i < 3; ++i)
        s.centre[i] = (eyes[0].position[i] + eyes[1].position[i]) * 0.5f - s.normal[i] * d;
    s.halfW = d * s_hudTanX;
    s.halfH = d * s_hudTanY;
    return s;
}

// Flat virtual screen as submitted: fixed in tracking space, 16:9.
PointerSurface ScreenSurface()
{
    const PoseidonVR::VRConfig& cfg = PoseidonVR::VRSession::Instance().Config();
    PointerSurface s = {{0, 0, -cfg.screenDistance}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, cfg.screenWidth * 0.5f,
                        cfg.screenWidth * 0.5f * 9.0f / 16.0f};
    return s;
}

// The right controller works as a mouse: its ray places the UI cursor and
// the right trigger clicks.  Active wherever there is UI to click and no
// gun to fire: menus, map, the death/mission-end screens (spectator), the
// dev panel, and every flat frame (menus without a 3D scene, loading).
void UpdatePointer(const PoseidonVR::HandPose& aim, const PointerSurface& surface, bool active, unsigned int windowId,
                   int winW, int winH)
{
    float x = 0, y = 0;
    if (!active || !aim.valid || !PointAt(aim, surface, x, y))
    {
        if (!active)
            s_pointerDown = false;
        return;
    }
    Poseidon::InputSubsystem::Instance().SetPointerCursor(x, y);

    // The dev panel (ImGui) tracks the OS mouse position from motion events;
    // no relative motion, so nothing turns or aims.
    SDL_Event motion = {};
    motion.type = SDL_EVENT_MOUSE_MOTION;
    motion.motion.windowID = windowId;
    motion.motion.x = (x + 1) * 0.5f * winW;
    motion.motion.y = (y + 1) * 0.5f * winH;
    SDL_PushEvent(&motion);

    const float trigger = PoseidonVR::VRSession::Instance().Controllers().trigger[1];
    const bool down = s_pointerDown ? trigger > 0.4f : trigger > 0.6f;
    if (down != s_pointerDown)
    {
        s_pointerDown = down;
        SDL_Event ev = {};
        ev.type = down ? SDL_EVENT_MOUSE_BUTTON_DOWN : SDL_EVENT_MOUSE_BUTTON_UP;
        ev.button.windowID = windowId;
        ev.button.button = SDL_BUTTON_LEFT;
        ev.button.down = down;
        ev.button.clicks = 1;
        ev.button.x = (x + 1) * 0.5f * winW;
        ev.button.y = (y + 1) * 0.5f * winH;
        SDL_PushEvent(&ev);
    }
}

// The weapon points along the right controller's aim ray, its grip in the
// right hand.  With the left grip held near the weapon, the barrel instead
// runs from the right hand through the left one (two-handed hold).
void UpdateWeaponPose()
{
    s_weaponValid = false;
    s_twoHanded = false;
    const PoseidonVR::HandPose& aim = s_frame.aim[1];
    const PoseidonVR::HandPose& grip = s_frame.grip[1];
    if (!aim.valid || !grip.valid)
        return;

    auto& vr = PoseidonVR::VRSession::Instance();
    const PoseidonVR::VRConfig& cfg = vr.Config();
    Matrix3 orient = s_bodyYaw * ToEngineOrientation(aim.orientation);
    const Vector3 hand = TrackedToWorld(grip.position);

    // Two-handed hold: the left grip takes the fore-end only when the left
    // hand is at it (foreend_radius from where the one-handed weapon's
    // fore-end is), then holds it while the grip stays pressed.  Not while
    // the left hand is busy reloading.
    const PoseidonVR::HandPose& support = s_frame.grip[0];
    const float leftGrip = vr.Controllers().grip[0];
    const bool canHold = cfg.twoHanded >= 0.5f && support.valid && !s_magInHand && !s_charging;
    if (!canHold || leftGrip < 0.4f)
    {
        s_foreGrip = false;
    }
    else if (!s_foreGrip && leftGrip > 0.6f)
    {
        const Vector3 centre = hand + orient.Direction() * cfg.weaponForward + orient.DirectionUp() * cfg.weaponUp +
                               orient.DirectionAside() * cfg.weaponRight;
        const Vector3 foreEnd = centre + orient.Direction() * cfg.foreEndForward;
        // VR (M6): a grip nearer the magazine well than the fore-end is
        // for the magazine (UpdateReload), not the fore-end.
        const Vector3 well = centre + orient.Direction() * cfg.wellForward - orient.DirectionUp() * cfg.wellDown;
        const Vector3 supportPos = TrackedToWorld(support.position);
        // Nor when the handle waits for the hand and is nearer (some sit far
        // forward).
        const float toHandle = (supportPos - ChargeHandle(orient, centre)).Size();
        const bool forHandle = !s_chambered && s_magLoaded && toHandle < cfg.chargeRadius &&
                               toHandle < (supportPos - foreEnd).Size();
        s_foreGrip = !forHandle && (supportPos - foreEnd).Size() < cfg.foreEndRadius &&
                     (supportPos - foreEnd).Size() < (supportPos - well).Size();
    }
    if (s_foreGrip)
    {
        Vector3 dir = TrackedToWorld(support.position) - hand;
        const float len = dir.Size();
        if (len > 0.12f && len < 0.9f)
        {
            orient.SetDirectionAndUp(dir * (1.0f / len), orient.DirectionUp());
            s_twoHanded = true;
        }
    }

    s_weaponWorld.SetOrientation(orient);
    s_weaponWorld.SetPosition(hand + orient.Direction() * cfg.weaponForward + orient.DirectionUp() * cfg.weaponUp +
                              orient.DirectionAside() * cfg.weaponRight);
    s_weaponValid = true;
}

// Manual reload (M6), as gestures with the left hand:
//   B (right)            eject the magazine: it drops and falls (speed reload)
//   grip at the well     pull the magazine out into the hand (tactical reload)
//   grip at the pouch    take a magazine (only with the weapon empty and a
//                        spare left)
//   let go at the pouch  put the magazine in the hand back
//   let go elsewhere     drop it: it falls
//   bring it to the well insert it
//   grip at the handle   pull it back along the barrel to the end and let
//                        go: it snaps forward and chambers a round (needed
//                        after every magazine, charge_every_magazine)
// Each step vibrates the hand doing it.  The game side (magazines, firing)
// is in Man, which takes the events through VRTakeReloadEvent.
void UpdateReload(const float headXr[3])
{
    auto& vr = PoseidonVR::VRSession::Instance();
    const PoseidonVR::VRConfig& cfg = vr.Config();
    const PoseidonVR::ControllerState& c = vr.Controllers();

    const bool ejectPress = c.b && !s_ejectHeld;
    s_ejectHeld = c.b;
    const bool gripNow = s_leftGripHeld ? c.grip[0] > 0.4f : c.grip[0] > 0.6f;
    const bool gripPress = gripNow && !s_leftGripHeld;
    s_leftGripHeld = gripNow;

    const PoseidonVR::HandPose& leftGrip = s_frame.grip[0];
    if (cfg.manualReload < 0.5f || !s_weaponValid || !s_magStateValid || s_uiInput || !leftGrip.valid)
    {
        if (ejectPress)
        {
            char line[160];
            std::snprintf(line, sizeof(line),
                          "Reload: B ignored (manual %d, weapon tracked %d, soldier reported %d, UI %d (dialog %d, "
                          "context %d), left hand %d)",
                          cfg.manualReload >= 0.5f, s_weaponValid, s_magStateValid, s_uiInput, s_dialogOpen,
                          static_cast<int>(Poseidon::InputSubsystem::Instance().GetContext()), leftGrip.valid);
            vr.LogLine(line);
        }
        if (s_magInHand)
        {
            PushReloadEvent(Engine::VRReloadStow); // interrupted: the magazine stays in the pouch
            vr.LogLine("Reload: hand interrupted, magazine back in the pouch");
        }
        s_magInHand = s_charging = s_boltBack = false;
        s_boltPull = 0.0f;
        s_magStateValid = false;
        return;
    }
    const Vector3 left = TrackedToWorld(leftGrip.position);
    const Matrix4& weapon = s_weaponWorld;

    // Left hand velocity, smoothed, for a magazine let go while moving.
    float frameTime = 0.0f;
    {
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - s_leftPrevTime).count();
        frameTime = std::min(dt, 0.05f);
        if (dt > 0.001f && dt < 0.2f)
            s_leftVelocity = s_leftVelocity * 0.5f + (left - s_leftPrev) * (0.5f / dt);
        else
            s_leftVelocity = VZero;
        s_leftPrev = left;
        s_leftPrevTime = now;
    }
    Matrix4 leftHand;
    leftHand.SetOrientation(s_bodyYaw * ToEngineOrientation(s_frame.aim[0].orientation));
    leftHand.SetPosition(left);
    const Vector3 well = weapon.Position() + weapon.Direction() * cfg.wellForward - weapon.DirectionUp() * cfg.wellDown;

    if (ejectPress)
    {
        if (s_magLoaded && !s_magInHand)
        {
            PushReloadEvent(Engine::VRReloadEject, MagazineInWell(weapon), weapon.DirectionUp() * -1.0f);
            vr.Haptic(1, 0.5f, 0.05f);
            Flash("Magazine out");
        }
        vr.LogLine(s_magLoaded ? "Reload: eject" : "Reload: B with no magazine in");
    }

    // Pouch: on the chest, placed from the head along the body's heading.
    const Matrix3 body = s_bodyId ? YawMatrix(s_bodyHeading) : s_bodyYaw;
    const Vector3 head = TrackedToWorld(headXr);
    const Vector3 pouch =
        head + body.Direction() * cfg.pouchForward - VUp * cfg.pouchDown + body.DirectionAside() * cfg.pouchRight;

    // Left grip held + left stick click: the pouch is where the hand is now.
    const bool pouchSet = c.thumbClick[0] && !s_pouchSetHeld;
    s_pouchSetHeld = c.thumbClick[0];
    if (pouchSet && gripNow && !s_magInHand && !s_charging)
    {
        const Vector3 d = left - head;
        PoseidonVR::VRConfig& edit = vr.MutableConfig();
        edit.pouchForward = d * body.Direction();
        edit.pouchRight = d * body.DirectionAside();
        edit.pouchDown = -(d * VUp);
        vr.SaveConfig();
        vr.Haptic(0, 0.7f, 0.15f);
        Flash("Pouch set here");
        char line[128];
        std::snprintf(line, sizeof(line), "Reload: pouch set at forward %.2f, right %.2f, down %.2f m",
                      edit.pouchForward, edit.pouchRight, edit.pouchDown);
        vr.LogLine(line);
    }
    // The handle waits for the hand: a grip nearer it than the well is for it.
    const Vector3 handle = ChargeHandle(weapon.Orientation(), weapon.Position());
    const float toHandle = (left - handle).Size();
    const bool atHandle = s_magLoaded && !s_chambered && toHandle < cfg.chargeRadius && toHandle < (left - well).Size();

    // Grip at the well with a magazine in: pull it out into the hand.
    if (!s_magInHand && !s_charging && !s_foreGrip && !atHandle && gripPress && s_magLoaded &&
        (left - well).Size() < cfg.wellRadius)
    {
        s_magInHand = true;
        s_insertArmed = false; // must leave the well before it can go back in
        PushReloadEvent(Engine::VRReloadUnload);
        vr.Haptic(0, 0.6f, 0.06f);
        Flash("Magazine out (in hand)");
        vr.LogLine("Reload: magazine pulled out by hand");
    }
    else if (!s_magInHand && gripPress && !s_magLoaded)
    {
        const float dist = (left - pouch).Size();
        char line[128];
        if (s_magSpares > 0 && dist < cfg.pouchRadius)
        {
            s_magInHand = true;
            s_insertArmed = true;
            PushReloadEvent(Engine::VRReloadTake);
            vr.Haptic(0, 0.6f, 0.06f);
            Flash("Magazine taken");
            std::snprintf(line, sizeof(line), "Reload: magazine taken from the pouch (%.2f m from it)", dist);
        }
        else
        {
            std::snprintf(line, sizeof(line), "Reload: left grip %.2f m from the pouch (radius %.2f), %d spare", dist,
                          cfg.pouchRadius, s_magSpares);
        }
        vr.LogLine(line);
    }

    if (s_magInHand)
    {
        if (!s_insertArmed && (left - well).Size() > cfg.wellRadius * 1.5f)
            s_insertArmed = true;
        if (s_insertArmed && !s_magLoaded && (left - well).Size() < cfg.wellRadius)
        {
            PushReloadEvent(Engine::VRReloadInsert);
            s_magInHand = false;
            vr.Haptic(0, 0.8f, 0.08f);
            vr.Haptic(1, 0.5f, 0.06f);
            vr.LogLine("Reload: magazine inserted");
            Flash("Magazine in");
        }
        else if (!gripNow)
        {
            // Let go: at the pouch it goes back in, anywhere else it falls.
            s_magInHand = false;
            const float toPouch = (left - pouch).Size();
            char line[128];
            if (toPouch < cfg.pouchRadius)
            {
                PushReloadEvent(Engine::VRReloadStow);
                vr.Haptic(0, 0.4f, 0.04f);
                Flash("Magazine in the pouch");
                std::snprintf(line, sizeof(line), "Reload: magazine back in the pouch (%.2f m from it)", toPouch);
            }
            else
            {
                PushReloadEvent(Engine::VRReloadDrop, MagazineInHand(leftHand, false), s_leftVelocity);
                Flash("Magazine dropped");
                std::snprintf(line, sizeof(line), "Reload: magazine dropped %.2f m from the pouch, %.2f m from the well",
                              toPouch, (left - well).Size());
            }
            vr.LogLine(line);
        }
    }

    // Charging handle: follows the hand back along the weapon; let go, it
    // snaps forward.  A round goes in only when it went all the way back.
    if (s_magLoaded && !s_chambered && !s_magInHand)
    {
        const float travel = s_handleValid ? s_handleTravel : cfg.chargePull;
        const float along = (left - weapon.Position()) * weapon.Direction();
        if (!s_charging && gripPress && !s_foreGrip)
        {
            char line[128];
            if (atHandle && s_boltPull <= 0.0f)
            {
                s_charging = true;
                s_chargeStart = along;
                vr.Haptic(0, 0.3f, 0.03f);
                std::snprintf(line, sizeof(line), "Reload: handle taken (%.2f m from it, travel %.2f m)", toHandle, travel);
            }
            else
            {
                std::snprintf(line, sizeof(line), "Reload: left grip %.2f m from the handle (radius %.2f), %.2f m from the well",
                              toHandle, cfg.chargeRadius, (left - well).Size());
            }
            vr.LogLine(line);
        }
        if (s_charging)
        {
            if (!gripNow)
            {
                s_charging = false;
                vr.LogLine(s_boltBack ? "Reload: handle let go from the back"
                                      : "Reload: handle let go short of the end, nothing chambered");
            }
            else
            {
                s_boltPull = std::clamp((s_chargeStart - along) / travel, 0.0f, 1.0f);
                if (s_boltPull >= 0.9f && !s_boltBack)
                {
                    s_boltBack = true;
                    vr.Haptic(0, 0.6f, 0.04f);
                }
            }
        }
    }
    else
    {
        s_charging = false;
    }
    if (!s_charging && s_boltPull > 0.0f)
        s_boltPull = std::max(s_boltPull - frameTime * 12.0f, 0.0f);
    if (s_boltBack && s_boltPull < 0.05f)
    {
        s_boltBack = false;
        PushReloadEvent(Engine::VRReloadCharge);
        vr.Haptic(0, 0.9f, 0.08f);
        vr.Haptic(1, 0.5f, 0.05f);
        vr.LogLine("Reload: charged");
        Flash("Charged");
    }

    // Reported again by the soldier's next simulation step; until then
    // (dead, other camera) no gestures.
    s_magStateValid = false;
}

// Raw byte copies between framebuffers: no scissor, no sRGB encoding of an
// already gamma-encoded frame, all channels.  Restores the state it touched
// (the colour mask is all-RGBA everywhere in the engine, see BeginPass).
struct RawBlitScope
{
    GLboolean scissor = glIsEnabled(GL_SCISSOR_TEST);
    GLboolean srgb = glIsEnabled(GL_FRAMEBUFFER_SRGB);

    RawBlitScope()
    {
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_FRAMEBUFFER_SRGB);
        Poseidon::render::pipeline::SetColorMask(true);
    }
    ~RawBlitScope()
    {
        if (srgb)
            glEnable(GL_FRAMEBUFFER_SRGB);
        if (scissor)
            glEnable(GL_SCISSOR_TEST);
    }
};

// Largest rectangle of the source's aspect centred in the destination.
void FitRect(int srcW, int srcH, int dstX, int dstY, int dstW, int dstH, int rect[4])
{
    int w = dstW, h = dstH;
    if (srcW > 0 && srcH > 0)
    {
        if (srcW * dstH > srcH * dstW)
            h = dstW * srcH / srcW;
        else
            w = dstH * srcW / srcH;
    }
    rect[0] = dstX + (dstW - w) / 2;
    rect[1] = dstY + (dstH - h) / 2;
    rect[2] = rect[0] + w;
    rect[3] = rect[1] + h;
}

// Reload status at the bottom of the HUD, large, every gameplay frame: what
// the weapon needs next, plus a short confirmation after each step.  The
// game's own message line sits in a corner of the HUD, 30 degrees off
// centre in the headset, where it is easily missed.
void DrawVRStatus()
{
    const bool reloading = PoseidonVR::VRSession::Instance().Config().manualReload >= 0.5f && s_weaponValid &&
                           !s_uiInput;
    const char* hint = nullptr;
    if (reloading)
    {
        if (s_magInHand)
            hint = "Magazine in hand - push it in under the weapon";
        else if (!s_magLoaded)
            hint = s_magSpares > 0 ? "NO MAGAZINE - left grip at your chest" : "NO MAGAZINE - none left";
        else if (!s_chambered)
            hint = "Pull the charging handle all the way back and let go";
    }
    const bool flash = s_flashText && std::chrono::steady_clock::now() < s_flashUntil;
    if (!hint && !flash)
        return;

    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float size = display.y * 0.035f;
    ImDrawList* draw = ImGui::GetForegroundDrawList();
    float y = display.y * 0.78f;
    for (const char* text : {flash ? s_flashText : nullptr, hint})
    {
        if (!text)
            continue;
        const ImVec2 extent = ImGui::GetFont()->CalcTextSizeA(size, FLT_MAX, 0.0f, text);
        const ImVec2 pos((display.x - extent.x) * 0.5f, y);
        draw->AddRectFilled(ImVec2(pos.x - 12, pos.y - 6), ImVec2(pos.x + extent.x + 12, pos.y + extent.y + 6),
                            IM_COL32(0, 0, 0, 160), 6.0f);
        draw->AddText(ImGui::GetFont(), size, pos, text == hint ? IM_COL32(255, 210, 80, 255) : IM_COL32_WHITE,
                      text);
        y += extent.y + 18;
    }
}

// Dev panel "VR" tab (--dev, Ctrl+`).  In a stereo frame the panel is drawn
// onto the HUD layer, so it can be used from inside the headset.
void DrawVRTab()
{
    auto& vr = PoseidonVR::VRSession::Instance();
    PoseidonVR::VRConfig& c = vr.MutableConfig();

    ImGui::Text("%s", vr.IsFake() ? "Fake headset (--vr-fake)" : "OpenXR session");
    if (s_eyes[0].fbo)
        ImGui::Text("Eye targets: %dx%d, %dx MSAA", s_eyes[0].width, s_eyes[0].height, s_eyes[0].samples);

    ImGui::SeparatorText("HUD");
    ImGui::SliderFloat("Half width (tan)", &c.hudHalfWidthTan, 0.3f, 1.2f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Angular size of the HUD/menus. Smaller = text nearer the centre of view.");
    ImGui::SliderFloat("Distance (m)", &c.hudDistance, 0.5f, 5.0f, "%.2f");

    ImGui::SeparatorText("Flat screen (loading, menus without 3D)");
    ImGui::SliderFloat("Screen distance (m)", &c.screenDistance, 1.0f, 10.0f, "%.2f");
    ImGui::SliderFloat("Screen width (m)", &c.screenWidth, 1.0f, 10.0f, "%.2f");

    ImGui::SeparatorText("Controls");
    bool smooth = c.smoothTurn >= 0.5f;
    if (ImGui::Checkbox("Smooth turn", &smooth))
        c.smoothTurn = smooth ? 1.0f : 0.0f;
    if (smooth)
        ImGui::SliderFloat("Turn speed (deg/s)", &c.smoothSpeed, 30.0f, 360.0f, "%.0f");
    else
        ImGui::SliderFloat("Snap angle (deg)", &c.snapAngle, 10.0f, 90.0f, "%.0f");
    int locomotion = static_cast<int>(c.locomotion + 0.5f);
    ImGui::TextUnformatted("Left stick forward is:");
    ImGui::RadioButton("Body", &locomotion, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Head", &locomotion, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Left controller", &locomotion, 2);
    c.locomotion = static_cast<float>(locomotion);
    const PoseidonVR::ControllerState& pad = vr.Controllers();
    ImGui::TextDisabled("Sticks L %.2f %.2f  R %.2f %.2f  Triggers %.2f %.2f  Grips %.2f %.2f%s", pad.moveX,
                        pad.moveY, pad.turnX, pad.turnY, pad.trigger[0], pad.trigger[1], pad.grip[0], pad.grip[1],
                        pad.active ? "" : "  (no input focus)");
    ImGui::TextDisabled("Right stick click: recentre (stand up straight for it)");

    ImGui::SeparatorText("Two-handed hold");
    ImGui::SliderFloat("Fore-end grab radius (m)", &c.foreEndRadius, 0.05f, 0.40f, "%.2f");
    ImGui::SliderFloat("Fore-end position (m)", &c.foreEndForward, 0.0f, 0.6f, "%.2f");
    ImGui::TextDisabled("Left hand %s", s_foreGrip ? "holds the fore-end" : "free");

    ImGui::SeparatorText("Weapon zeroing");
    ImGui::SliderFloat("Zero yaw (deg, + right)", &c.zeroYaw, -3.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Zero pitch (deg, + up)", &c.zeroPitch, -3.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Muzzle shift right (cm)", &c.muzzleRight, -10.0f, 10.0f, "%.1f");
    ImGui::SliderFloat("Muzzle shift up (cm)", &c.muzzleUp, -10.0f, 10.0f, "%.1f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Angles: hits move more the farther the target. Shift: same offset at every distance.");

    ImGui::SeparatorText("Manual reload");
    bool manual = c.manualReload >= 0.5f;
    if (ImGui::Checkbox("Reload by hand (B ejects)", &manual))
        c.manualReload = manual ? 1.0f : 0.0f;
    ImGui::SliderFloat("Pouch below head (m)", &c.pouchDown, 0.2f, 0.9f, "%.2f");
    ImGui::SliderFloat("Pouch ahead (m)", &c.pouchForward, 0.0f, 0.4f, "%.2f");
    ImGui::SliderFloat("Pouch right (m, - left)", &c.pouchRight, -0.4f, 0.4f, "%.2f");
    bool everyMagazine = c.chargeEveryMagazine >= 0.5f;
    if (ImGui::Checkbox("Every magazine needs the charging handle", &everyMagazine))
        c.chargeEveryMagazine = everyMagazine ? 1.0f : 0.0f;
    ImGui::SliderFloat("Charge pull (m)", &c.chargePull, 0.02f, 0.15f, "%.2f");
    ImGui::SliderFloat("Magazine in hand ahead (m)", &c.magForward, -0.15f, 0.15f, "%.2f");
    ImGui::SliderFloat("Magazine in hand below (m)", &c.magDown, -0.10f, 0.20f, "%.2f");
    ImGui::SliderFloat("Magazine in hand tilt (deg)", &c.magPitch, -180.0f, 180.0f, "%.0f");
    ImGui::SliderFloat("Magazine model scale", &c.magScale, 0.2f, 3.0f, "%.2f");
    ImGui::TextDisabled("Weapon: %s, %d spare, %s%s%s", s_magLoaded ? "magazine in" : "EMPTY", s_magSpares,
                        s_chambered ? "chambered" : "needs charge", s_magInHand ? ", magazine in hand" : "",
                        s_charging ? ", pulling handle" : "");

    ImGui::SeparatorText("Physical stance");
    bool physical = c.physicalStance >= 0.5f;
    if (ImGui::Checkbox("Crouch / lie down for real", &physical))
        c.physicalStance = physical ? 1.0f : 0.0f;
    ImGui::SliderFloat("Prone at drop (m)", &c.proneDrop, 0.4f, 1.2f, "%.2f");
    ImGui::SliderFloat("Eye height (m)", &c.eyeHeight, -0.3f, 0.3f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Fine-tune the standing view height (it already follows your real height above the floor).");
    if (s_frame.headHeight > 0.0f)
        ImGui::TextDisabled("Eyes above your floor: %.2f m", s_frame.headHeight);
    else
        ImGui::TextDisabled("Runtime gives no floor level: standing view = the soldier's eyes");
    ImGui::SliderFloat("Free lean (m)", &c.leanFree, 0.0f, 0.3f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How far the head may move sideways/forward before the body stays under it.");
    if (s_headDropValid)
        ImGui::TextDisabled("Head drop now: %.2f m", s_headDrop);

    ImGui::SeparatorText("Render");
    ImGui::SliderFloat("Eye scale", &c.eyeScale, 0.3f, 1.5f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Per-eye resolution vs the headset's recommendation. Applies after a restart.");

    ImGui::Spacing();
    if (ImGui::Button("Save to vr.ini"))
        vr.SaveConfig();
    ImGui::TextDisabled("vr.ini is also re-read when edited on disk.");
}

} // namespace

void EngineGL33::InitVR()
{
    const auto& cfg = Poseidon::Foundation::AppConfig::Instance();
    if (!cfg.IsVREnabled() || !_glContext)
        return;

    auto& vr = PoseidonVR::VRSession::Instance();
    _vrActive = cfg.IsVRFake() ? vr.InitFake() : vr.Init();
    if (!_vrActive)
    {
        LOG_WARN(Graphics, "VR: OpenXR unavailable, continuing flat (see vr.log)");
        return;
    }
    s_mirror = cfg.IsVRMirror();
    Poseidon::Dev::DebugOverlay::SetExtraTab("VR", DrawVRTab);
    Poseidon::Dev::DebugOverlay::SetExtraOverlay(DrawVRStatus);
    // Test aid (CWR_VR_DECOY_PAD=1): a silent pad attached before ours, as
    // Virtual Desktop's gamepad emulation does, to check the game still
    // reads the controllers.
    if (std::getenv("CWR_VR_DECOY_PAD") && SDL_InitSubSystem(SDL_INIT_GAMEPAD))
    {
        SDL_VirtualJoystickDesc decoy;
        SDL_INIT_INTERFACE(&decoy);
        decoy.type = SDL_JOYSTICK_TYPE_GAMEPAD;
        decoy.naxes = SDL_GAMEPAD_AXIS_COUNT;
        decoy.nbuttons = SDL_GAMEPAD_BUTTON_COUNT;
        decoy.name = "Decoy pad";
        if (SDL_JoystickID id = SDL_AttachVirtualJoystick(&decoy))
        {
            SDL_OpenJoystick(id);
            LOG_INFO(Graphics, "VR: decoy gamepad attached (test)");
        }
    }
    // Also with --vr-fake, so the scripted controllers reach the game the
    // same way the real ones do.
    if (s_gamepad.Create())
        LOG_INFO(Graphics, "VR: controllers attached as a virtual gamepad");
    else
        LOG_WARN(Graphics, "VR: virtual gamepad failed ({}), controllers unavailable", SDL_GetError());
    // xrWaitFrame paces the loop from now on (see SetSwapInterval).
    if (!vr.IsFake())
        SDL_GL_SetSwapInterval(0);
    LOG_INFO(Graphics, "VR: {} session ready (see vr.log)", vr.IsFake() ? "fake" : "OpenXR");
}

void EngineGL33::ShutdownVR()
{
    if (_vrActive)
    {
        Poseidon::Dev::DebugOverlay::SetExtraTab(nullptr, nullptr);
        Poseidon::Dev::DebugOverlay::SetExtraOverlay(nullptr);
    }
    s_gamepad.Destroy();
    s_aimValid = false;
    s_weaponValid = false;
    s_headDropValid = false;
    for (EyeTarget& t : s_eyes)
        DestroyEyeTarget(t);
    if (_vrBlitFbo)
    {
        glDeleteFramebuffers(1, &_vrBlitFbo);
        _vrBlitFbo = 0;
    }
    if (_vrActive)
        PoseidonVR::VRSession::Instance().Shutdown();
    _vrActive = false;
    _vrEyeTarget = -1;
}

int EngineGL33::VRScenePassCount()
{
    _vrStereoFrame = false;
    _vrEyeTarget = -1;
    if (!_vrActive)
        return 1;

    _vrFrameAttempted = true;
    if (!PoseidonVR::VRSession::Instance().BeginStereoFrame(s_frame))
    {
        // No head pose this frame: leave aiming and the weapon to the game.
        s_aimValid = false;
        s_weaponValid = false;
        s_headDropValid = false;
        return 1;
    }
    // The XR frame is begun: from here it is submitted in SubmitVRFrame even
    // if an eye target fails (that eye then shows nothing).
    for (EyeTarget& t : s_eyes)
        EnsureEyeTarget(t, s_frame.eyeWidth, s_frame.eyeHeight, _msaaSamples);
    _vrStereoFrame = true;
    return 2;
}

void EngineGL33::VRBeginScenePass(int pass, Camera& camera)
{
    if (!_vrStereoFrame)
        return;

    if (pass == 0)
    {
        s_gameCamera = camera.Transform();
        s_near = camera.Near();
        s_far = camera.Far();
        // The game camera gives the position; the heading is VR's own
        // (UpdateHeading) and pitch/roll come from the head, so the mouse
        // never tilts the world.
        const auto& eyes = s_frame.eyes;
        const Vector3 headPos = (ToEnginePosition(eyes[0].position) + ToEnginePosition(eyes[1].position)) * 0.5f;
        const PoseidonVR::HandPose& leftAim = s_frame.aim[0];
        UpdateHeading(HeadingOf(ToEngineOrientation(eyes[0].orientation)), headPos, HeadingOf(s_gameCamera.Orientation()),
                      s_gameCamera.Position(), leftAim.valid,
                      leftAim.valid ? HeadingOf(ToEngineOrientation(leftAim.orientation)) : 0.0f);
        UpdateHeadDrop(headPos);
        s_bodyYaw = YawMatrix(s_playerYaw);
        UpdateWeaponPose();
        const auto ctx = Poseidon::InputSubsystem::Instance().GetContext();
        const bool wasUi = s_uiInput;
        s_uiInput = s_dialogOpen || ctx == Poseidon::InputContext::Menu || ctx == Poseidon::InputContext::Map ||
                    ctx == Poseidon::InputContext::Spectator || Poseidon::Dev::DebugOverlay::IsVisible();
        if (s_uiInput != wasUi)
        {
            char line[96];
            std::snprintf(line, sizeof(line), "Pointer %s (dialog %d, input context %d)", s_uiInput ? "on" : "off",
                          s_dialogOpen, static_cast<int>(ctx));
            PoseidonVR::VRSession::Instance().LogLine(line);
        }
        UpdatePointer(s_frame.aim[1], HudSurface(), s_uiInput, _sdlWindow ? SDL_GetWindowID(_sdlWindow) : 0, _w, _h);
        const float headXr[3] = {(eyes[0].position[0] + eyes[1].position[0]) * 0.5f,
                                 (eyes[0].position[1] + eyes[1].position[1]) * 0.5f,
                                 (eyes[0].position[2] + eyes[1].position[2]) * 0.5f};
        UpdateReload(headXr);
    }

    // Whatever is still queued belongs to the previous target.
    BeginScreenPass();
    FlushAndFreeAllQueues(_queueNo);

    const PoseidonVR::EyeView& eye = s_frame.eyes[pass];
    PlaceCamera(camera, ToEngineOrientation(eye.orientation), TrackedOffset(eye.position));
    camera.SetPerspective(s_near, s_far, 1.0f, 1.0f);
    camera.SetAsymmetricFrustum(eye.tanLeft, eye.tanRight, eye.tanDown, eye.tanUp);
    camera.Adjust(this);

    _vrEyeTarget = pass;
    BindFrameRenderTarget();
    // Clear colour is still the fog colour InitDraw set.
    Poseidon::render::clear::ColorDepthStencil();
}

void EngineGL33::VRBeginHUDPass(Camera& camera)
{
    if (!_vrStereoFrame)
        return;

    BeginScreenPass();
    FlushAndFreeAllQueues(_queueNo);

    _vrEyeTarget = -1;
    BindFrameRenderTarget();
    // Transparent: the HUD layer is alpha-blended over the eyes.
    Poseidon::render::pipeline::SetClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    Poseidon::render::clear::ColorDepthStencil();

    // HUD camera: between the eyes, FOV equal to the HUD quad's angular size
    // so markers projected by the HUD sit on top of their targets.
    const auto& eyes = s_frame.eyes;
    const float centre[3] = {(eyes[0].position[0] + eyes[1].position[0]) * 0.5f,
                             (eyes[0].position[1] + eyes[1].position[1]) * 0.5f,
                             (eyes[0].position[2] + eyes[1].position[2]) * 0.5f};
    s_hudTanX = PoseidonVR::VRSession::Instance().Config().hudHalfWidthTan;
    s_hudTanY = _w > 0 ? s_hudTanX * _h / _w : s_hudTanX;
    const Matrix3 head = ToEngineOrientation(eyes[0].orientation);
    PlaceCamera(camera, head, TrackedOffset(centre));
    camera.SetPerspective(s_near, s_far, s_hudTanX, s_hudTanY);
    camera.Adjust(this);

    // The game aims along the barrel, or along the view without a tracked
    // hand (read back by World next frame).
    s_aimDir = s_weaponValid ? s_weaponWorld.Direction() : (s_bodyYaw * head).Direction();
    s_aimValid = true;
}

bool EngineGL33::VRAimDirection(Vector3& dir) const
{
    if (!_vrActive || !s_aimValid)
        return false;
    dir = s_aimDir;
    return true;
}

bool EngineGL33::VRWeaponTwoHanded() const
{
    return _vrActive && s_weaponValid && s_twoHanded;
}

bool EngineGL33::VRHandPose(int hand, Matrix4& world) const
{
    if (!_vrActive || !s_weaponValid || hand < 0 || hand > 1 || !s_frame.grip[hand].valid ||
        !s_frame.aim[hand].valid)
        return false;
    world.SetOrientation(s_bodyYaw * ToEngineOrientation(s_frame.aim[hand].orientation));
    world.SetPosition(TrackedToWorld(s_frame.grip[hand].position));
    return true;
}

float EngineGL33::VRForeEndForward() const
{
    return PoseidonVR::VRSession::Instance().Config().foreEndForward;
}

bool EngineGL33::VRMagazineInHandPose(Matrix4& world) const
{
    Matrix4 hand;
    if (!s_magInHand || !VRManualReload() || !VRHandPose(0, hand))
        return false;
    world = MagazineInHand(hand, true);
    return true;
}

bool EngineGL33::VRWeaponPose(Matrix4& world, bool firing) const
{
    if (!_vrActive || !s_weaponValid)
        return false;
    world = s_weaponWorld;
    if (firing)
    {
        // Zeroing turns and shifts the barrel line only (the drawn weapon
        // stays as held): angles move hits more with distance, the origin
        // shift moves them by the same amount everywhere.
        const PoseidonVR::VRConfig& cfg = PoseidonVR::VRSession::Instance().Config();
        world.SetPosition(world.Position() + world.DirectionAside() * (cfg.muzzleRight * 0.01f) +
                          world.DirectionUp() * (cfg.muzzleUp * 0.01f));
        Vector3 dir = world.Direction() + world.DirectionAside() * std::tan(cfg.zeroYaw * kDegToRad) +
                      world.DirectionUp() * std::tan(cfg.zeroPitch * kDegToRad);
        Matrix3 orient;
        orient.SetDirectionAndUp(dir, world.DirectionUp());
        world.SetOrientation(orient);
    }
    return true;
}

bool EngineGL33::VRHeadDrop(float& metres) const
{
    if (!_vrActive || !s_headDropValid || PoseidonVR::VRSession::Instance().Config().physicalStance < 0.5f)
        return false;
    metres = s_headDrop;
    return true;
}

void EngineGL33::VRSetDialogOpen(bool open)
{
    s_dialogOpen = open;
}

bool EngineGL33::VRManualReload() const
{
    return _vrActive && s_weaponValid && PoseidonVR::VRSession::Instance().Config().manualReload >= 0.5f;
}

void EngineGL33::VRSetMagazineState(bool loaded, int spares, bool chambered, int weaponKey)
{
    if (weaponKey != s_weaponKey)
    {
        // Weapon switched: a magazine in the hand goes back to the pouch.
        if (s_magInHand)
        {
            PushReloadEvent(VRReloadStow);
            PoseidonVR::VRSession::Instance().LogLine("Reload: weapon switched, magazine back in the pouch");
        }
        s_magInHand = s_charging = s_boltBack = s_handleValid = false;
        s_boltPull = 0.0f;
        s_weaponKey = weaponKey;
    }
    s_magStateValid = true;
    s_magLoaded = loaded;
    s_magSpares = spares;
    s_chambered = chambered;
}

bool EngineGL33::VRChargeEveryMagazine() const
{
    return PoseidonVR::VRSession::Instance().Config().chargeEveryMagazine >= 0.5f;
}

void EngineGL33::VRSetChargeHandle(Vector3Par world, float travel)
{
    if (!s_weaponValid)
        return;
    const Vector3 d = world - s_weaponWorld.Position();
    const Vector3 local(d * s_weaponWorld.Direction(), d * s_weaponWorld.DirectionUp(), d * s_weaponWorld.DirectionAside());
    if (!s_handleValid)
    {
        char line[128];
        std::snprintf(line, sizeof(line), "Reload: charging handle %.3f ahead, %.3f up, %.3f right of the weapon centre, travel %.2f",
                      local.X(), local.Y(), local.Z(), travel);
        PoseidonVR::VRSession::Instance().LogLine(line);
    }
    s_handleValid = true;
    s_handleLocal = local;
    s_handleTravel = travel;
}

float EngineGL33::VRChargePull() const
{
    return s_boltPull;
}

int EngineGL33::VRTakeReloadEvent()
{
    if (s_eventCount == 0)
        return VRReloadNone;
    s_lastEvent = s_events[0];
    for (int i = 1; i < s_eventCount; ++i)
        s_events[i - 1] = s_events[i];
    --s_eventCount;
    return s_lastEvent.kind;
}

bool EngineGL33::VRReloadEventPose(Matrix4& world, Vector3& velocity) const
{
    if (s_lastEvent.kind != VRReloadEject && s_lastEvent.kind != VRReloadDrop)
        return false;
    world = s_lastEvent.pose;
    velocity = s_lastEvent.velocity;
    return true;
}

void EngineGL33::VRSetPlayerBody(const void* id, const Matrix3& orientation, bool soldier, float groundY)
{
    s_bodyId = id;
    s_bodyHeading = HeadingOf(orientation);
    s_bodySoldier = soldier;
    s_bodyGroundY = groundY;
}

float EngineGL33::VRProneDrop() const
{
    return PoseidonVR::VRSession::Instance().Config().proneDrop;
}

void EngineGL33::VRSetProne(bool prone)
{
    s_gameProne = prone;
}

void EngineGL33::BindVREyeTarget()
{
    const EyeTarget& t = s_eyes[_vrEyeTarget];
    glBindFramebuffer(GL_FRAMEBUFFER, t.fbo);
    glViewport(0, 0, t.width, t.height);
}

void EngineGL33::VREyeTargetSize(int& w, int& h) const
{
    w = s_eyes[_vrEyeTarget].width;
    h = s_eyes[_vrEyeTarget].height;
}

void EngineGL33::SubmitVRFrame()
{
    if (!_vrActive)
        return;
    auto& vr = PoseidonVR::VRSession::Instance();
    vr.ReloadConfigIfChanged();
    // Read by the engine's input pass at the start of the next frame.  B is
    // the eject button in gameplay with manual reload, the gamepad's B in UI.
    const bool forwardB = s_uiInput || vr.Config().manualReload < 0.5f;
    s_gamepad.Update(vr.Controllers(), s_moveYaw, forwardB);
    // Every 5 s: the controllers as OpenXR gives them and what the game read
    // from the virtual pad (movement and firing go that way).
    {
        static auto s_nextInputLog = std::chrono::steady_clock::now();
        const auto now = std::chrono::steady_clock::now();
        if (now >= s_nextInputLog)
        {
            s_nextInputLog = now + std::chrono::seconds(5);
            const PoseidonVR::ControllerState& c = vr.Controllers();
            char game[256];
            SDLInput_GamepadDebug(game, sizeof(game));
            char line[400];
            std::snprintf(line, sizeof(line), "Input: controllers move %.2f %.2f trigger %.2f (virtual pad %s, UI %d); %s",
                          c.moveX, c.moveY, c.trigger[1], s_gamepad.IsCreated() ? "on" : "MISSING", s_uiInput, game);
            vr.LogLine(line);
        }
    }
    if (_vrStereoFrame)
        SubmitVRStereoFrame();
    else if (!_vrFrameAttempted)
        SubmitVRFlatFrame();
    // A failed stereo attempt has already ended its XR frame.
    _vrStereoFrame = false;
    _vrFrameAttempted = false;
    _vrEyeTarget = -1;
    // World reports the body and dialogs again next frame.
    s_bodyId = nullptr;
    s_dialogOpen = false;
}

void EngineGL33::SubmitVRStereoFrame()
{
    RawBlitScope raw;
    if (!_vrBlitFbo)
        glGenFramebuffers(1, &_vrBlitFbo);

    auto blitToTexture = [&](GLuint srcFbo, int srcW, int srcH, GLuint tex, int dstW, int dstH)
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _vrBlitFbo);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, srcFbo);
        glBlitFramebuffer(0, 0, srcW, srcH, 0, 0, dstW, dstH, GL_COLOR_BUFFER_BIT, GL_LINEAR);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    };

    // HUD: the resolved window frame target still has the alpha the HUD
    // pass drew (the default framebuffer may have none).  Stretched to the
    // swapchain; the quad's size restores the aspect.
    if (s_frame.hudTexture)
    {
        int winW = _w, winH = _h;
        if (_sdlWindow)
            SDL_GetWindowSizeInPixels(_sdlWindow, &winW, &winH);
        const GLuint src = SSAAActive() ? _ssaaResolveFbo : 0;
        const int srcW = SSAAActive() ? _ssaaW : winW;
        const int srcH = SSAAActive() ? _ssaaH : winH;
        blitToTexture(src, srcW, srcH, s_frame.hudTexture, s_frame.hudWidth, s_frame.hudHeight);
    }

    for (int i = 0; i < 2; ++i)
    {
        const EyeTarget& t = s_eyes[i];
        if (!t.fbo)
            continue;
        glBindFramebuffer(GL_READ_FRAMEBUFFER, t.fbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, t.resolveFbo);
        glBlitFramebuffer(0, 0, t.width, t.height, 0, 0, t.width, t.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        if (s_frame.eyeTexture[i])
            blitToTexture(t.resolveFbo, t.width, t.height, s_frame.eyeTexture[i], s_frame.eyeWidth,
                          s_frame.eyeHeight);
    }

    // Window: the left eye, or both side by side with --vr-mirror.
    int winW = _w, winH = _h;
    if (_sdlWindow)
        SDL_GetWindowSizeInPixels(_sdlWindow, &winW, &winH);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    Poseidon::render::pipeline::SetClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    Poseidon::render::clear::WithMask(GL_COLOR_BUFFER_BIT);
    const int shown = s_mirror ? 2 : 1;
    for (int i = 0; i < shown; ++i)
    {
        const EyeTarget& t = s_eyes[i];
        if (!t.resolveFbo)
            continue;
        int rect[4];
        FitRect(t.width, t.height, i * winW / shown, 0, winW / shown, winH, rect);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, t.resolveFbo);
        glBlitFramebuffer(0, 0, t.width, t.height, rect[0], rect[1], rect[2], rect[3], GL_COLOR_BUFFER_BIT,
                          GL_LINEAR);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    PoseidonVR::VRSession::Instance().EndStereoFrame(s_hudTanX, s_hudTanY);
}

void EngineGL33::SubmitVRFlatFrame()
{
    auto& vr = PoseidonVR::VRSession::Instance();
    PoseidonVR::FlatFrameTarget target;
    if (!vr.BeginFlatFrame(target))
        return;

    int winW = _w, winH = _h;
    if (_sdlWindow)
        SDL_GetWindowSizeInPixels(_sdlWindow, &winW, &winH);

    // Flat frames are all UI (menus without a 3D scene, loading): point and
    // click on the virtual screen.  The window is 16:9 under VR, so it fills
    // the screen and screen coordinates are window coordinates.
    s_uiInput = true;
    UpdatePointer(vr.Aim(1), ScreenSurface(), true, _sdlWindow ? SDL_GetWindowID(_sdlWindow) : 0, _w, _h);

    RawBlitScope raw;
    if (!_vrBlitFbo)
        glGenFramebuffers(1, &_vrBlitFbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _vrBlitFbo);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, target.glTexture, 0);
    // Fit the window into the 16:9 screen without stretching; the bars stay
    // black.
    Poseidon::render::pipeline::SetClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    Poseidon::render::clear::WithMask(GL_COLOR_BUFFER_BIT);
    int rect[4];
    FitRect(winW, winH, 0, 0, target.width, target.height, rect);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, winW, winH, rect[0], rect[1], rect[2], rect[3], GL_COLOR_BUFFER_BIT, GL_LINEAR);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    vr.EndFlatFrame();
}
