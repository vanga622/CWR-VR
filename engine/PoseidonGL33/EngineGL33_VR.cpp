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
void UpdateHeading(float headYaw, const Vector3& headPos, float gameYaw, const Vector3& cameraPos)
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
        vr.LogLine(recentre ? "Recentred (right stick click)" : "Recentred (first stereo frame)");
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

    const bool bodyChanged = s_bodyId != s_lastBodyId;
    const float jump = (cameraPos - s_lastCameraPos).Size();
    if (bodyChanged || jump > 10.0f) // 10 m in one frame: faster than any vehicle
    {
        const float diff = WrapAngle(gameYaw - (s_playerYaw + headYaw));
        s_playerYaw = WrapAngle(gameYaw - headYaw);
        char line[128];
        std::snprintf(line, sizeof(line), "View re-synced to the game camera (%s, %.0f deg)",
                      bodyChanged ? "new body" : "camera jump", diff / kDegToRad);
        LOG_INFO(Graphics, "VR: {}", line);
        vr.LogLine(line);
    }
    s_lastBodyId = s_bodyId;
    s_lastCameraPos = cameraPos;

    // The game moves the soldier relative to its body; turn the stick from
    // the view (or keep it on the body) accordingly.
    const float bodyYaw = s_bodyId ? s_bodyHeading : gameYaw;
    s_moveYaw = cfg.headLocomotion >= 0.5f ? WrapAngle(s_playerYaw + headYaw - bodyYaw) : 0.0f;
}

// Weapon in the right hand (M4).
bool s_weaponValid = false;
bool s_twoHanded = false;
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

    // TrackedOffset's y is headPos - origin; shift it to the height wanted.
    const float fromOrigin = headPos.Y() - s_headOrigin.Y();
    float wanted = fromOrigin;
    if (cfg.physicalStance >= 0.5f)
        wanted = s_gameProne ? 0.0f : belowStand;
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

    const PoseidonVR::HandPose& support = s_frame.grip[0];
    if (cfg.twoHanded >= 0.5f && vr.Controllers().grip[0] > 0.5f && support.valid)
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
    bool headMove = c.headLocomotion >= 0.5f;
    if (ImGui::Checkbox("Walk where I look", &headMove))
        c.headLocomotion = headMove ? 1.0f : 0.0f;
    const PoseidonVR::ControllerState& pad = vr.Controllers();
    ImGui::TextDisabled("Sticks L %.2f %.2f  R %.2f %.2f  Triggers %.2f %.2f  Grips %.2f %.2f%s", pad.moveX,
                        pad.moveY, pad.turnX, pad.turnY, pad.trigger[0], pad.trigger[1], pad.grip[0], pad.grip[1],
                        pad.active ? "" : "  (no input focus)");
    ImGui::TextDisabled("Right stick click: recentre (stand up straight for it)");

    ImGui::SeparatorText("Weapon zeroing");
    ImGui::SliderFloat("Zero yaw (deg, + right)", &c.zeroYaw, -3.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Zero pitch (deg, + up)", &c.zeroPitch, -3.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Muzzle shift right (cm)", &c.muzzleRight, -10.0f, 10.0f, "%.1f");
    ImGui::SliderFloat("Muzzle shift up (cm)", &c.muzzleUp, -10.0f, 10.0f, "%.1f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Angles: hits move more the farther the target. Shift: same offset at every distance.");

    ImGui::SeparatorText("Physical stance");
    bool physical = c.physicalStance >= 0.5f;
    if (ImGui::Checkbox("Crouch / lie down for real", &physical))
        c.physicalStance = physical ? 1.0f : 0.0f;
    ImGui::SliderFloat("Prone at drop (m)", &c.proneDrop, 0.4f, 1.2f, "%.2f");
    ImGui::SliderFloat("Eye height (m)", &c.eyeHeight, -0.3f, 0.3f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Raise if standing upright still feels crouched (the soldier is shorter than you).");
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
    if (!vr.IsFake())
    {
        if (s_gamepad.Create())
            LOG_INFO(Graphics, "VR: controllers attached as a virtual gamepad");
        else
            LOG_WARN(Graphics, "VR: virtual gamepad failed ({}), controllers unavailable", SDL_GetError());
    }
    // xrWaitFrame paces the loop from now on (see SetSwapInterval).
    if (!vr.IsFake())
        SDL_GL_SetSwapInterval(0);
    LOG_INFO(Graphics, "VR: {} session ready (see vr.log)", vr.IsFake() ? "fake" : "OpenXR");
}

void EngineGL33::ShutdownVR()
{
    if (_vrActive)
        Poseidon::Dev::DebugOverlay::SetExtraTab(nullptr, nullptr);
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
        UpdateHeading(HeadingOf(ToEngineOrientation(eyes[0].orientation)), headPos, HeadingOf(s_gameCamera.Orientation()),
                      s_gameCamera.Position());
        UpdateHeadDrop(headPos);
        s_bodyYaw = YawMatrix(s_playerYaw);
        UpdateWeaponPose();
        const auto ctx = Poseidon::InputSubsystem::Instance().GetContext();
        const bool uiInput = ctx == Poseidon::InputContext::Menu || ctx == Poseidon::InputContext::Map ||
                             ctx == Poseidon::InputContext::Spectator || Poseidon::Dev::DebugOverlay::IsVisible();
        UpdatePointer(s_frame.aim[1], HudSurface(), uiInput, _sdlWindow ? SDL_GetWindowID(_sdlWindow) : 0, _w, _h);
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

void EngineGL33::VRSetPlayerBody(const void* id, const Matrix3& orientation)
{
    s_bodyId = id;
    s_bodyHeading = HeadingOf(orientation);
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
    // Read by the engine's input pass at the start of the next frame.
    s_gamepad.Update(vr.Controllers(), s_moveYaw);
    if (_vrStereoFrame)
        SubmitVRStereoFrame();
    else if (!_vrFrameAttempted)
        SubmitVRFlatFrame();
    // A failed stereo attempt has already ended its XR frame.
    _vrStereoFrame = false;
    _vrFrameAttempted = false;
    _vrEyeTarget = -1;
    // World reports the body again next frame if the camera is still on it.
    s_bodyId = nullptr;
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
