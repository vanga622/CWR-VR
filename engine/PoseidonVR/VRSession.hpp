#pragma once

// CWR-VR: OpenXR session for the Poseidon engine.
//
// M1 scope: one OpenXR session on the game's own OpenGL context
// (XR_KHR_opengl_enable) that shows the finished flat frame on a quad layer
// floating in front of the player.  The library makes no GL calls itself: it
// hands the graphics backend a swapchain texture, the backend blits the frame
// into it, and the session submits the layer.
//
// Everything is logged to vr.log next to the executable.  Any failure (no
// runtime, no headset, missing extension) leaves the session inactive and the
// game keeps running flat.

namespace PoseidonVR
{

// Swapchain image the backend must fill with the current flat frame.
struct FlatFrameTarget
{
    unsigned int glTexture = 0; // GL_TEXTURE_2D name owned by the runtime
    int width = 0;
    int height = 0;
};

class VRSession
{
public:
    static VRSession& Instance();

    // Create the instance, session and quad swapchain.  The game's GL context
    // must be current on the calling thread.  Returns false (and logs why)
    // when VR is unavailable.
    bool Init();
    void Shutdown();

    // True between a successful Init and Shutdown (the session may still be
    // idle, waiting for the runtime to make it ready).
    bool IsActive() const;

    // Once per presented frame, before SwapWindow: poll runtime events and,
    // while the session runs, wait for/begin an XR frame.  Returns true when
    // `out` holds an acquired swapchain image the caller must fill and then
    // hand back with EndFlatFrame.  Returns false when there is nothing to
    // draw this frame (the XR frame, if any, is already ended).
    bool BeginFlatFrame(FlatFrameTarget& out);
    void EndFlatFrame();

private:
    VRSession() = default;
    VRSession(const VRSession&) = delete;
    VRSession& operator=(const VRSession&) = delete;
};

} // namespace PoseidonVR
