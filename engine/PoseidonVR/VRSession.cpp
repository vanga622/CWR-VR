#include <PoseidonVR/VRSession.hpp>

#include <cstdarg>
#include <cstdio>
#include <ctime>

#if defined(_WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <unknwn.h> // IUnknown, used by openxr_platform.h; cut by WIN32_LEAN_AND_MEAN
#include <GL/gl.h>

#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_OPENGL
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#endif

namespace PoseidonVR
{
namespace
{

// --- vr.log ---------------------------------------------------------------

FILE* s_log = nullptr;

void OpenLog()
{
    if (s_log)
        return;
#if defined(_WIN32)
    // Next to the executable, not in the working directory (that is the
    // game data folder).
    wchar_t path[MAX_PATH] = {};
    DWORD len = GetModuleFileNameW(nullptr, path, MAX_PATH);
    while (len > 0 && path[len - 1] != L'\\' && path[len - 1] != L'/')
        --len;
    path[len] = 0;
    wcscat_s(path, L"vr.log");
    s_log = _wfopen(path, L"w");
#else
    s_log = std::fopen("vr.log", "w");
#endif
}

void CloseLog()
{
    if (s_log)
    {
        std::fclose(s_log);
        s_log = nullptr;
    }
}

void Log(const char* fmt, ...)
{
    if (!s_log)
        return;
    std::time_t now = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &now);
#else
    localtime_r(&now, &tm);
#endif
    char stamp[16];
    std::strftime(stamp, sizeof(stamp), "%H:%M:%S", &tm);
    std::fprintf(s_log, "[%s] ", stamp);

    va_list args;
    va_start(args, fmt);
    std::vfprintf(s_log, fmt, args);
    va_end(args);
    std::fputc('\n', s_log);
    std::fflush(s_log);
}

#if defined(_WIN32)

// --- OpenXR state ---------------------------------------------------------

// Virtual screen: 16:9, 3.2 m wide, 2.5 m in front of the initial head pose.
// TODO(M2): move to vr.ini with the other tunables.
constexpr int kScreenPixelsW = 1920;
constexpr int kScreenPixelsH = 1080;
constexpr float kScreenMetersW = 3.2f;
constexpr float kScreenMetersH = 1.8f;
constexpr float kScreenDistance = 2.5f;

constexpr int64_t kGL_RGBA8 = 0x8058;
constexpr int64_t kGL_SRGB8_ALPHA8 = 0x8C43;

struct State
{
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId systemId = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace localSpace = XR_NULL_HANDLE;
    XrSwapchain swapchain = XR_NULL_HANDLE;
    std::vector<XrSwapchainImageOpenGLKHR> images;
    int swapW = 0;
    int swapH = 0;
    XrEnvironmentBlendMode blendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    XrSessionState sessionState = XR_SESSION_STATE_UNKNOWN;
    bool running = false;
    bool frameBegun = false;
    bool imageAcquired = false;
    XrTime predictedDisplayTime = 0;

    // Frame timing, summarised to vr.log every kStatsFrames frames.
    static constexpr int kStatsFrames = 600;
    int statsFrames = 0;
    int statsSkipped = 0;
    double statsWaitMs = 0.0;
    double statsPeriodMs = 0.0;
};

State s;

const char* ResultName(XrResult r)
{
    static char buf[XR_MAX_RESULT_STRING_SIZE];
    if (s.instance != XR_NULL_HANDLE && XR_SUCCEEDED(xrResultToString(s.instance, r, buf)))
        return buf;
    std::snprintf(buf, sizeof(buf), "XrResult(%d)", static_cast<int>(r));
    return buf;
}

bool Check(XrResult r, const char* what)
{
    if (XR_SUCCEEDED(r))
        return true;
    Log("ERROR: %s failed: %s", what, ResultName(r));
    return false;
}

const char* SessionStateName(XrSessionState st)
{
    switch (st)
    {
        case XR_SESSION_STATE_IDLE: return "IDLE";
        case XR_SESSION_STATE_READY: return "READY";
        case XR_SESSION_STATE_SYNCHRONIZED: return "SYNCHRONIZED";
        case XR_SESSION_STATE_VISIBLE: return "VISIBLE";
        case XR_SESSION_STATE_FOCUSED: return "FOCUSED";
        case XR_SESSION_STATE_STOPPING: return "STOPPING";
        case XR_SESSION_STATE_LOSS_PENDING: return "LOSS_PENDING";
        case XR_SESSION_STATE_EXITING: return "EXITING";
        default: return "UNKNOWN";
    }
}

std::string VersionString(XrVersion v)
{
    return std::to_string(XR_VERSION_MAJOR(v)) + "." + std::to_string(XR_VERSION_MINOR(v)) + "." +
           std::to_string(XR_VERSION_PATCH(v));
}

void LogApiLayers()
{
    uint32_t count = 0;
    if (!Check(xrEnumerateApiLayerProperties(0, &count, nullptr), "xrEnumerateApiLayerProperties"))
        return;
    std::vector<XrApiLayerProperties> layers(count, {XR_TYPE_API_LAYER_PROPERTIES});
    xrEnumerateApiLayerProperties(count, &count, layers.data());
    Log("API layers: %u", count);
    for (const auto& l : layers)
        Log("  layer %s (spec %s, v%u)", l.layerName, VersionString(l.specVersion).c_str(), l.layerVersion);
}

// Logs every instance extension the runtime offers; returns whether the GL
// one is among them.
bool LogExtensionsAndCheckGL()
{
    uint32_t count = 0;
    if (!Check(xrEnumerateInstanceExtensionProperties(nullptr, 0, &count, nullptr),
               "xrEnumerateInstanceExtensionProperties"))
        return false;
    std::vector<XrExtensionProperties> exts(count, {XR_TYPE_EXTENSION_PROPERTIES});
    xrEnumerateInstanceExtensionProperties(nullptr, count, &count, exts.data());

    bool hasGL = false;
    Log("Runtime extensions: %u", count);
    for (const auto& e : exts)
    {
        Log("  %s v%u", e.extensionName, e.extensionVersion);
        if (std::strcmp(e.extensionName, XR_KHR_OPENGL_ENABLE_EXTENSION_NAME) == 0)
            hasGL = true;
    }
    return hasGL;
}

void LogSystem()
{
    XrSystemProperties props{XR_TYPE_SYSTEM_PROPERTIES};
    if (!Check(xrGetSystemProperties(s.instance, s.systemId, &props), "xrGetSystemProperties"))
        return;
    Log("System: \"%s\" vendor 0x%x, max swapchain %ux%u, max layers %u, tracking orientation=%d position=%d",
        props.systemName, props.vendorId, props.graphicsProperties.maxSwapchainImageWidth,
        props.graphicsProperties.maxSwapchainImageHeight, props.graphicsProperties.maxLayerCount,
        props.trackingProperties.orientationTracking, props.trackingProperties.positionTracking);

    // Stereo view sizes, for M2.
    uint32_t viewCount = 0;
    if (XR_SUCCEEDED(xrEnumerateViewConfigurationViews(s.instance, s.systemId,
                                                       XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &viewCount,
                                                       nullptr)))
    {
        std::vector<XrViewConfigurationView> views(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
        xrEnumerateViewConfigurationViews(s.instance, s.systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                          viewCount, &viewCount, views.data());
        for (uint32_t i = 0; i < viewCount; ++i)
            Log("  view %u: recommended %ux%u (max %ux%u), samples %u", i, views[i].recommendedImageRectWidth,
                views[i].recommendedImageRectHeight, views[i].maxImageRectWidth, views[i].maxImageRectHeight,
                views[i].recommendedSwapchainSampleCount);
    }

    uint32_t blendCount = 0;
    if (XR_SUCCEEDED(xrEnumerateEnvironmentBlendModes(s.instance, s.systemId,
                                                      XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &blendCount,
                                                      nullptr)) &&
        blendCount > 0)
    {
        std::vector<XrEnvironmentBlendMode> modes(blendCount);
        xrEnumerateEnvironmentBlendModes(s.instance, s.systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                         blendCount, &blendCount, modes.data());
        s.blendMode = modes[0];
        for (auto m : modes)
            Log("  blend mode %d", static_cast<int>(m));
    }
}

bool CheckGLRequirements()
{
    PFN_xrGetOpenGLGraphicsRequirementsKHR getReqs = nullptr;
    if (!Check(xrGetInstanceProcAddr(s.instance, "xrGetOpenGLGraphicsRequirementsKHR",
                                     reinterpret_cast<PFN_xrVoidFunction*>(&getReqs)),
               "xrGetInstanceProcAddr(xrGetOpenGLGraphicsRequirementsKHR)"))
        return false;

    XrGraphicsRequirementsOpenGLKHR reqs{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
    // Mandatory before xrCreateSession, even if we only log the result.
    if (!Check(getReqs(s.instance, s.systemId, &reqs), "xrGetOpenGLGraphicsRequirementsKHR"))
        return false;

    const char* glVersion = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    int major = 0, minor = 0;
    if (glVersion)
        std::sscanf(glVersion, "%d.%d", &major, &minor);
    Log("OpenGL: context \"%s\", runtime wants %s .. %s", glVersion ? glVersion : "?",
        VersionString(reqs.minApiVersionSupported).c_str(), VersionString(reqs.maxApiVersionSupported).c_str());
    if (XR_MAKE_VERSION(major, minor, 0) < reqs.minApiVersionSupported)
        Log("WARNING: context version is below the runtime minimum; session creation may fail");
    return true;
}

int64_t PickSwapchainFormat()
{
    uint32_t count = 0;
    if (!Check(xrEnumerateSwapchainFormats(s.session, 0, &count, nullptr), "xrEnumerateSwapchainFormats"))
        return 0;
    std::vector<int64_t> formats(count);
    xrEnumerateSwapchainFormats(s.session, count, &count, formats.data());

    std::string list;
    for (auto f : formats)
    {
        char hex[16];
        std::snprintf(hex, sizeof(hex), " 0x%llx", static_cast<unsigned long long>(f));
        list += hex;
    }
    Log("Swapchain formats (%u):%s", count, list.c_str());

    // The flat frame is already gamma-encoded RGBA8.  An sRGB swapchain
    // stores those bytes unchanged (raw blit) and the compositor decodes them
    // correctly; a linear RGBA8 swapchain would show it too bright.
    for (int64_t want : {kGL_SRGB8_ALPHA8, kGL_RGBA8})
        for (auto f : formats)
            if (f == want)
                return f;
    return formats.empty() ? 0 : formats[0];
}

bool CreateQuadSwapchain()
{
    const int64_t format = PickSwapchainFormat();
    if (format == 0)
        return false;

    XrSwapchainCreateInfo info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
                      XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    info.format = format;
    info.sampleCount = 1;
    info.width = kScreenPixelsW;
    info.height = kScreenPixelsH;
    info.faceCount = 1;
    info.arraySize = 1;
    info.mipCount = 1;
    if (!Check(xrCreateSwapchain(s.session, &info, &s.swapchain), "xrCreateSwapchain"))
        return false;

    uint32_t count = 0;
    xrEnumerateSwapchainImages(s.swapchain, 0, &count, nullptr);
    s.images.assign(count, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});
    if (!Check(xrEnumerateSwapchainImages(s.swapchain, count, &count,
                                          reinterpret_cast<XrSwapchainImageBaseHeader*>(s.images.data())),
               "xrEnumerateSwapchainImages"))
        return false;

    s.swapW = kScreenPixelsW;
    s.swapH = kScreenPixelsH;
    Log("Quad swapchain: %dx%d format 0x%llx, %u images, %.1fx%.1f m at %.1f m", s.swapW, s.swapH,
        static_cast<unsigned long long>(format), count, kScreenMetersW, kScreenMetersH, kScreenDistance);
    return true;
}

void LogReferenceSpaces()
{
    uint32_t count = 0;
    if (XR_FAILED(xrEnumerateReferenceSpaces(s.session, 0, &count, nullptr)))
        return;
    std::vector<XrReferenceSpaceType> spaces(count);
    xrEnumerateReferenceSpaces(s.session, count, &count, spaces.data());
    std::string list;
    for (auto sp : spaces)
        list += sp == XR_REFERENCE_SPACE_TYPE_VIEW    ? " VIEW"
                : sp == XR_REFERENCE_SPACE_TYPE_LOCAL ? " LOCAL"
                : sp == XR_REFERENCE_SPACE_TYPE_STAGE ? " STAGE"
                                                      : " other";
    Log("Reference spaces:%s", list.c_str());
}

void DestroyAll()
{
    if (s.swapchain != XR_NULL_HANDLE)
        xrDestroySwapchain(s.swapchain);
    if (s.localSpace != XR_NULL_HANDLE)
        xrDestroySpace(s.localSpace);
    if (s.session != XR_NULL_HANDLE)
        xrDestroySession(s.session);
    if (s.instance != XR_NULL_HANDLE)
        xrDestroyInstance(s.instance);
    s = State{};
}

void HandleSessionState(XrSessionState st)
{
    Log("Session state: %s -> %s", SessionStateName(s.sessionState), SessionStateName(st));
    s.sessionState = st;
    switch (st)
    {
        case XR_SESSION_STATE_READY:
        {
            XrSessionBeginInfo begin{XR_TYPE_SESSION_BEGIN_INFO};
            begin.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            s.running = Check(xrBeginSession(s.session, &begin), "xrBeginSession");
            break;
        }
        case XR_SESSION_STATE_STOPPING:
            s.running = false;
            Check(xrEndSession(s.session), "xrEndSession");
            break;
        case XR_SESSION_STATE_EXITING:
        case XR_SESSION_STATE_LOSS_PENDING:
            // The runtime is going away (headset disconnected, runtime
            // closed).  Drop VR, keep playing flat.
            Log("Session ending; VR disabled for the rest of this run");
            DestroyAll();
            break;
        default:
            break;
    }
}

void PollEvents()
{
    for (;;)
    {
        if (s.instance == XR_NULL_HANDLE)
            return;
        XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
        XrResult r = xrPollEvent(s.instance, &ev);
        if (r != XR_SUCCESS)
            return;
        switch (ev.type)
        {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED:
                HandleSessionState(reinterpret_cast<XrEventDataSessionStateChanged&>(ev).state);
                break;
            case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
                Log("Instance loss pending; VR disabled for the rest of this run");
                DestroyAll();
                return;
            case XR_TYPE_EVENT_DATA_EVENTS_LOST:
                Log("WARNING: %u runtime events lost",
                    reinterpret_cast<XrEventDataEventsLost&>(ev).lostEventCount);
                break;
            default:
                Log("Event type %d", static_cast<int>(ev.type));
                break;
        }
    }
}

void EndFrame(bool withLayer)
{
    XrCompositionLayerQuad quad{XR_TYPE_COMPOSITION_LAYER_QUAD};
    const XrCompositionLayerBaseHeader* layers[] = {reinterpret_cast<XrCompositionLayerBaseHeader*>(&quad)};
    if (withLayer)
    {
        quad.space = s.localSpace;
        quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        quad.subImage.swapchain = s.swapchain;
        quad.subImage.imageRect = {{0, 0}, {s.swapW, s.swapH}};
        quad.subImage.imageArrayIndex = 0;
        quad.pose.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
        quad.pose.position = {0.0f, 0.0f, -kScreenDistance};
        quad.size = {kScreenMetersW, kScreenMetersH};
    }

    XrFrameEndInfo end{XR_TYPE_FRAME_END_INFO};
    end.displayTime = s.predictedDisplayTime;
    end.environmentBlendMode = s.blendMode;
    end.layerCount = withLayer ? 1 : 0;
    end.layers = withLayer ? layers : nullptr;
    Check(xrEndFrame(s.session, &end), "xrEndFrame");
    s.frameBegun = false;
}

#endif // _WIN32

} // namespace

VRSession& VRSession::Instance()
{
    static VRSession instance;
    return instance;
}

#if defined(_WIN32)

bool VRSession::Init()
{
    if (IsActive())
        return true;

    OpenLog();
    Log("CWR-VR: starting OpenXR (M1: flat game on a virtual screen)");

    LogApiLayers();
    if (!LogExtensionsAndCheckGL())
    {
        Log("ERROR: runtime has no %s. Fallbacks: SteamVR as the active runtime, or a D3D11 swapchain "
            "via WGL_NV_DX_interop2 (not implemented yet). Continuing flat.",
            XR_KHR_OPENGL_ENABLE_EXTENSION_NAME);
        return false;
    }

    const char* extensions[] = {XR_KHR_OPENGL_ENABLE_EXTENSION_NAME};
    XrInstanceCreateInfo ci{XR_TYPE_INSTANCE_CREATE_INFO};
    std::strncpy(ci.applicationInfo.applicationName, "CWR-VR", XR_MAX_APPLICATION_NAME_SIZE - 1);
    ci.applicationInfo.applicationVersion = 1;
    std::strncpy(ci.applicationInfo.engineName, "Poseidon", XR_MAX_ENGINE_NAME_SIZE - 1);
    ci.applicationInfo.engineVersion = 1;
    ci.applicationInfo.apiVersion = XR_API_VERSION_1_0;
    ci.enabledExtensionCount = 1;
    ci.enabledExtensionNames = extensions;
    if (!Check(xrCreateInstance(&ci, &s.instance), "xrCreateInstance"))
        return false;

    XrInstanceProperties ip{XR_TYPE_INSTANCE_PROPERTIES};
    if (Check(xrGetInstanceProperties(s.instance, &ip), "xrGetInstanceProperties"))
        Log("Runtime: %s %s", ip.runtimeName, VersionString(ip.runtimeVersion).c_str());

    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    if (!Check(xrGetSystem(s.instance, &sgi, &s.systemId), "xrGetSystem (is the headset connected?)"))
    {
        DestroyAll();
        return false;
    }
    LogSystem();

    if (!CheckGLRequirements())
    {
        DestroyAll();
        return false;
    }

    XrGraphicsBindingOpenGLWin32KHR binding{XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR};
    binding.hDC = wglGetCurrentDC();
    binding.hGLRC = wglGetCurrentContext();
    if (!binding.hDC || !binding.hGLRC)
    {
        Log("ERROR: no current WGL context on this thread");
        DestroyAll();
        return false;
    }

    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
    sci.next = &binding;
    sci.systemId = s.systemId;
    if (!Check(xrCreateSession(s.instance, &sci, &s.session), "xrCreateSession"))
    {
        DestroyAll();
        return false;
    }

    LogReferenceSpaces();
    XrReferenceSpaceCreateInfo rsi{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    rsi.poseInReferenceSpace.orientation.w = 1.0f;
    if (!Check(xrCreateReferenceSpace(s.session, &rsi, &s.localSpace), "xrCreateReferenceSpace(LOCAL)") ||
        !CreateQuadSwapchain())
    {
        DestroyAll();
        return false;
    }

    Log("Session created; waiting for the runtime to make it READY");
    return true;
}

void VRSession::Shutdown()
{
    if (s.instance == XR_NULL_HANDLE)
    {
        CloseLog();
        return;
    }
    if (s.imageAcquired)
    {
        XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        xrReleaseSwapchainImage(s.swapchain, &rel);
    }
    // xrEndSession is only legal in STOPPING; destroying a running session
    // is allowed and the runtime tears it down.
    DestroyAll();
    Log("Shutdown complete");
    CloseLog();
}

bool VRSession::IsActive() const
{
    return s.instance != XR_NULL_HANDLE;
}

bool VRSession::BeginFlatFrame(FlatFrameTarget& out)
{
    PollEvents();
    if (!s.running)
        return false;

    const auto waitStart = std::chrono::steady_clock::now();
    XrFrameState fs{XR_TYPE_FRAME_STATE};
    if (!Check(xrWaitFrame(s.session, nullptr, &fs), "xrWaitFrame"))
        return false;
    const double waitMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - waitStart).count();

    if (!Check(xrBeginFrame(s.session, nullptr), "xrBeginFrame"))
        return false;
    s.frameBegun = true;
    s.predictedDisplayTime = fs.predictedDisplayTime;

    s.statsFrames++;
    s.statsWaitMs += waitMs;
    s.statsPeriodMs += fs.predictedDisplayPeriod / 1.0e6;
    if (!fs.shouldRender)
        s.statsSkipped++;
    if (s.statsFrames == State::kStatsFrames)
    {
        Log("Frames: %d, avg xrWaitFrame %.2f ms, avg display period %.2f ms, not rendered %d", s.statsFrames,
            s.statsWaitMs / s.statsFrames, s.statsPeriodMs / s.statsFrames, s.statsSkipped);
        s.statsFrames = s.statsSkipped = 0;
        s.statsWaitMs = s.statsPeriodMs = 0.0;
    }

    if (!fs.shouldRender)
    {
        EndFrame(false);
        return false;
    }

    uint32_t index = 0;
    XrSwapchainImageAcquireInfo acq{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    if (!Check(xrAcquireSwapchainImage(s.swapchain, &acq, &index), "xrAcquireSwapchainImage"))
    {
        EndFrame(false);
        return false;
    }
    XrSwapchainImageWaitInfo wait{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    wait.timeout = XR_INFINITE_DURATION;
    if (!Check(xrWaitSwapchainImage(s.swapchain, &wait), "xrWaitSwapchainImage"))
    {
        XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        xrReleaseSwapchainImage(s.swapchain, &rel);
        EndFrame(false);
        return false;
    }
    s.imageAcquired = true;

    out.glTexture = s.images[index].image;
    out.width = s.swapW;
    out.height = s.swapH;
    return true;
}

void VRSession::EndFlatFrame()
{
    if (!s.frameBegun)
        return;
    bool haveImage = false;
    if (s.imageAcquired)
    {
        XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        haveImage = Check(xrReleaseSwapchainImage(s.swapchain, &rel), "xrReleaseSwapchainImage");
        s.imageAcquired = false;
    }
    EndFrame(haveImage);
}

#else // !_WIN32

bool VRSession::Init()
{
    OpenLog();
    Log("CWR-VR: VR is only implemented on Windows so far. Continuing flat.");
    CloseLog();
    return false;
}

void VRSession::Shutdown() {}

bool VRSession::IsActive() const
{
    return false;
}

bool VRSession::BeginFlatFrame(FlatFrameTarget&)
{
    return false;
}

void VRSession::EndFlatFrame() {}

#endif // _WIN32

} // namespace PoseidonVR
