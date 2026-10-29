// VR: OpenXR virtual screen for the GL33 backend (CWR-VR, milestone M1).
//
// PoseidonVR owns the OpenXR session and hands out swapchain textures; this
// file does the GL side — one blit of the finished window frame per present.

#include <PoseidonGL33/EngineGL33.hpp>
#include <Poseidon/Foundation/Platform/AppConfig.hpp>
#include <PoseidonVR/VRSession.hpp>

#include <SDL3/SDL.h>
#include <glad/gl.h>

void EngineGL33::InitVR()
{
    if (!Poseidon::Foundation::AppConfig::Instance().IsVREnabled() || !_glContext)
        return;

    _vrActive = PoseidonVR::VRSession::Instance().Init();
    if (!_vrActive)
    {
        LOG_WARN(Graphics, "VR: OpenXR unavailable, continuing flat (see vr.log)");
        return;
    }
    // xrWaitFrame paces the loop from now on (see SetSwapInterval).
    SDL_GL_SetSwapInterval(0);
    LOG_INFO(Graphics, "VR: OpenXR session created, flat frame goes to a virtual screen (see vr.log)");
}

void EngineGL33::ShutdownVR()
{
    if (_vrBlitFbo)
    {
        glDeleteFramebuffers(1, &_vrBlitFbo);
        _vrBlitFbo = 0;
    }
    if (_vrActive)
        PoseidonVR::VRSession::Instance().Shutdown();
    _vrActive = false;
}

void EngineGL33::SubmitVRFlatFrame()
{
    if (!_vrActive)
        return;

    auto& vr = PoseidonVR::VRSession::Instance();
    PoseidonVR::FlatFrameTarget target;
    if (!vr.BeginFlatFrame(target))
        return;

    int winW = _w, winH = _h;
    if (_sdlWindow)
        SDL_GetWindowSizeInPixels(_sdlWindow, &winW, &winH);

    // Fit the window into the 16:9 screen without stretching; the bars stay
    // black.
    int dstW = target.width;
    int dstH = target.height;
    if (winW > 0 && winH > 0)
    {
        if (winW * target.height > winH * target.width)
            dstH = target.width * winH / winW;
        else
            dstW = target.height * winW / winH;
    }
    const int dstX = (target.width - dstW) / 2;
    const int dstY = (target.height - dstH) / 2;

    // The blit must copy raw bytes: scissor would clip it, sRGB encoding would
    // gamma-correct an already gamma-corrected frame.
    const GLboolean scissor = glIsEnabled(GL_SCISSOR_TEST);
    const GLboolean srgb = glIsEnabled(GL_FRAMEBUFFER_SRGB);
    GLboolean colorMask[4];
    glGetBooleanv(GL_COLOR_WRITEMASK, colorMask);
    GLfloat clearColor[4];
    glGetFloatv(GL_COLOR_CLEAR_VALUE, clearColor);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    if (!_vrBlitFbo)
        glGenFramebuffers(1, &_vrBlitFbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _vrBlitFbo);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, target.glTexture, 0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, winW, winH, dstX, dstY, dstX + dstW, dstY + dstH, GL_COLOR_BUFFER_BIT, GL_LINEAR);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
    glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
    if (srgb)
        glEnable(GL_FRAMEBUFFER_SRGB);
    if (scissor)
        glEnable(GL_SCISSOR_TEST);

    vr.EndFlatFrame();
}
