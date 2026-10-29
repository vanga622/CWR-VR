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

#include <PoseidonGL33/EngineGL33.hpp>
#include <Poseidon/Foundation/Platform/AppConfig.hpp>
#include <Poseidon/Graphics/Core/GLClear.hpp>
#include <Poseidon/Graphics/Core/GLPipelineState.hpp>
#include <Poseidon/World/Scene/Camera/Camera.hpp>
#include <PoseidonVR/VRSession.hpp>

#include <SDL3/SDL.h>
#include <glad/gl.h>

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
    // xrWaitFrame paces the loop from now on (see SetSwapInterval).
    if (!vr.IsFake())
        SDL_GL_SetSwapInterval(0);
    LOG_INFO(Graphics, "VR: {} session ready (see vr.log)", vr.IsFake() ? "fake" : "OpenXR");
}

void EngineGL33::ShutdownVR()
{
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
        return 1;
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
        // Head tracking replaces the game's pitch and roll; only the body's
        // heading is kept, so mouse pitch never tilts the world.  Heading
        // from the aside vector (dir = aside x up), which stays horizontal
        // even when the game camera looks straight up or down.
        Vector3Val aside = s_gameCamera.DirectionAside();
        s_bodyYaw.SetUpAndDirection(VUp, Vector3(-aside.Z(), 0, aside.X()));
    }

    // Whatever is still queued belongs to the previous target.
    BeginScreenPass();
    FlushAndFreeAllQueues(_queueNo);

    const PoseidonVR::EyeView& eye = s_frame.eyes[pass];
    PlaceCamera(camera, ToEngineOrientation(eye.orientation), ToEnginePosition(eye.position));
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
    PlaceCamera(camera, ToEngineOrientation(eyes[0].orientation), ToEnginePosition(centre));
    camera.SetPerspective(s_near, s_far, s_hudTanX, s_hudTanY);
    camera.Adjust(this);
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
    if (_vrStereoFrame)
        SubmitVRStereoFrame();
    else if (!_vrFrameAttempted)
        SubmitVRFlatFrame();
    // A failed stereo attempt has already ended its XR frame.
    _vrStereoFrame = false;
    _vrFrameAttempted = false;
    _vrEyeTarget = -1;
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
