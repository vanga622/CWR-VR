#include <PoseidonVR/VRSession.hpp>

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

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
#include <vector>

#endif

namespace PoseidonVR
{
namespace
{

// --- Files next to the executable -------------------------------------------

std::string ExeDirFile(const char* name)
{
#if defined(_WIN32)
    // Next to the executable, not in the working directory (that is the
    // game data folder).
    char path[MAX_PATH] = {};
    DWORD len = GetModuleFileNameA(nullptr, path, MAX_PATH);
    while (len > 0 && path[len - 1] != '\\' && path[len - 1] != '/')
        --len;
    return std::string(path, len) + name;
#else
    return name;
#endif
}

// --- vr.log -------------------------------------------------------------------

FILE* s_log = nullptr;

void OpenLog()
{
    if (!s_log)
        s_log = std::fopen(ExeDirFile("vr.log").c_str(), "w");
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

// --- vr.ini -------------------------------------------------------------------

VRConfig s_config;

struct ConfigKey
{
    const char* section;
    const char* key;
    float VRConfig::*field;
    const char* comment;
};

const ConfigKey kConfigKeys[] = {
    {"render", "eye_scale", &VRConfig::eyeScale, "eye image size relative to the headset's recommendation"},
    {"hud", "distance", &VRConfig::hudDistance, "metres in front of the eyes"},
    {"hud", "half_width_tan", &VRConfig::hudHalfWidthTan, "horizontal half-size as tan(angle); 0.6 = 31 deg"},
    {"screen", "distance", &VRConfig::screenDistance, "flat virtual screen (menus, loading), metres"},
    {"screen", "width", &VRConfig::screenWidth, "flat virtual screen width, metres (16:9)"},
    {"controls", "smooth_turn", &VRConfig::smoothTurn, "right stick turning: 0 = snap, 1 = smooth"},
    {"controls", "snap_angle", &VRConfig::snapAngle, "degrees per snap turn"},
    {"controls", "smooth_speed", &VRConfig::smoothSpeed, "smooth turn speed, degrees per second"},
    {"controls", "head_locomotion", &VRConfig::headLocomotion,
     "1 = left stick moves where you look, 0 = where the body faces"},
    {"weapon", "offset_forward", &VRConfig::weaponForward, "weapon centre ahead of the right hand, metres"},
    {"weapon", "offset_up", &VRConfig::weaponUp, "weapon centre above the right hand, metres"},
    {"weapon", "offset_right", &VRConfig::weaponRight, "weapon centre right of the right hand, metres"},
    {"weapon", "two_handed", &VRConfig::twoHanded, "1 = left grip near the weapon aims it with both hands"},
    {"weapon", "zero_yaw", &VRConfig::zeroYaw, "zeroing, degrees: + moves hits right"},
    {"weapon", "zero_pitch", &VRConfig::zeroPitch, "zeroing, degrees: + moves hits up"},
    {"weapon", "muzzle_right", &VRConfig::muzzleRight, "shot origin shift, cm: + moves hits right at every distance"},
    {"weapon", "muzzle_up", &VRConfig::muzzleUp, "shot origin shift, cm: + moves hits up at every distance"},
    {"stance", "physical", &VRConfig::physicalStance,
     "1 = crouching lowers the view, lying down makes the soldier go prone"},
    {"stance", "prone_drop", &VRConfig::proneDrop, "head this far below standing height goes prone, metres"},
    {"stance", "eye_height", &VRConfig::eyeHeight, "metres added to the soldier's eye height (+ if you feel too short)"},
    {"stance", "lean_free", &VRConfig::leanFree,
     "sideways/forward head move kept, metres; beyond it the body stays under the head"},
    {"reload", "manual", &VRConfig::manualReload, "1 = B ejects, grab a magazine from the pouch, insert, charge"},
    {"reload", "pouch_forward", &VRConfig::pouchForward, "pouch ahead of the head, metres"},
    {"reload", "pouch_down", &VRConfig::pouchDown, "pouch below the head, metres"},
    {"reload", "pouch_right", &VRConfig::pouchRight, "pouch right of the head, metres (- = left)"},
    {"reload", "pouch_radius", &VRConfig::pouchRadius, "how close the left hand must be to grab, metres"},
    {"reload", "well_forward", &VRConfig::wellForward, "magazine well ahead of the weapon centre, metres"},
    {"reload", "well_down", &VRConfig::wellDown, "magazine well below the weapon centre, metres"},
    {"reload", "well_radius", &VRConfig::wellRadius, "how close the magazine must come to insert, metres"},
    {"reload", "charge_back", &VRConfig::chargeBack, "charging handle behind the weapon centre, metres"},
    {"reload", "charge_up", &VRConfig::chargeUp, "charging handle above the weapon centre, metres"},
    {"reload", "charge_radius", &VRConfig::chargeRadius, "how close the left hand must be to grab it, metres"},
    {"reload", "charge_pull", &VRConfig::chargePull, "pull back this far to charge, metres"},
};

std::filesystem::file_time_type s_configTime;
std::chrono::steady_clock::time_point s_configChecked;

std::filesystem::file_time_type ConfigFileTime(const std::string& path)
{
    std::error_code ec;
    const auto t = std::filesystem::last_write_time(path, ec);
    return ec ? std::filesystem::file_time_type{} : t;
}

void WriteConfig(const std::string& path, const VRConfig& values)
{
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f)
        return;
    std::fprintf(f, "; CWR-VR tunables, re-read while the game runs. Delete this file to get the defaults back.\n");
    const char* section = "";
    for (const auto& k : kConfigKeys)
    {
        if (std::strcmp(section, k.section) != 0)
        {
            section = k.section;
            std::fprintf(f, "\n[%s]\n", section);
        }
        std::fprintf(f, "; %s\n%s=%g\n", k.comment, k.key, values.*k.field);
    }
    std::fclose(f);
    s_configTime = ConfigFileTime(path);
}

void LoadConfig()
{
    s_config = VRConfig{};
    const std::string path = ExeDirFile("vr.ini");
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f)
    {
        WriteConfig(path, s_config);
        Log("vr.ini: not found, wrote defaults to %s", path.c_str());
        return;
    }
    s_configTime = ConfigFileTime(path);

    std::string section;
    char line[256];
    size_t found = 0;
    while (std::fgets(line, sizeof(line), f))
    {
        std::string s(line);
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
            s.pop_back();
        if (s.empty() || s[0] == ';' || s[0] == '#')
            continue;
        if (s[0] == '[')
        {
            section = s.substr(1, s.find(']') - 1);
            continue;
        }
        const size_t eq = s.find('=');
        if (eq == std::string::npos)
            continue;
        const std::string key = s.substr(0, eq);
        const float value = static_cast<float>(std::atof(s.c_str() + eq + 1));
        for (const auto& k : kConfigKeys)
            if (section == k.section && key == k.key)
            {
                s_config.*k.field = value;
                ++found;
            }
    }
    std::fclose(f);

    // A vr.ini from an older build lacks the newer keys: write them in
    // (with the values just read) so they can be found and edited.
    if (found < std::size(kConfigKeys))
    {
        WriteConfig(path, s_config);
        Log("vr.ini: added %d missing keys with default values",
            static_cast<int>(std::size(kConfigKeys) - found));
    }

    if (s_config.eyeScale < 0.2f || s_config.eyeScale > 2.0f)
        s_config.eyeScale = VRConfig{}.eyeScale;
    Log("vr.ini: eye_scale=%g hud distance=%g half_width_tan=%g screen distance=%g width=%g", s_config.eyeScale,
        s_config.hudDistance, s_config.hudHalfWidthTan, s_config.screenDistance, s_config.screenWidth);
}

// --- Fake headset -------------------------------------------------------------

bool s_fake = false;
ControllerState s_controllers;
HandPose s_aim[2], s_grip[2]; // latest, for flat frames as well as stereo
constexpr int kFakeEyeSize = 1440;
std::chrono::steady_clock::time_point s_fakeStart;

// Scripted head height for --vr-fake (env CWR_VR_FAKE_SCRIPT=stance): stand,
// crouch, stand, lie down, half up, stand, 4 s each, to check the physical
// stance without a headset.  0 otherwise.
float FakeHeadHeight()
{
    static const char* script = std::getenv("CWR_VR_FAKE_SCRIPT");
    if (!script || std::strcmp(script, "stance") != 0)
        return 0.0f;
    const float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - s_fakeStart).count();
    static const float kHeights[] = {0.0f, -0.45f, 0.0f, -0.90f, -0.30f, 0.0f};
    const int phase = static_cast<int>(t / 4.0f);
    return kHeights[phase < 6 ? phase : 5];
}

void FillFakeFrame(StereoFrame& out)
{
    const float headY = FakeHeadHeight();
    // Quest-2-like eyes: 64 mm IPD, slightly wider outward than inward.
    for (int i = 0; i < 2; ++i)
    {
        EyeView& e = out.eyes[i];
        const float side = i == 0 ? -1.0f : 1.0f;
        e = EyeView{};
        e.position[0] = side * 0.032f;
        e.position[1] = headY;
        e.tanLeft = i == 0 ? -1.0f : -0.84f;
        e.tanRight = i == 0 ? 0.84f : 1.0f;
        e.tanUp = 0.96f;
        e.tanDown = -1.1f;
    }
    // Scripted hands: rifle held at the hip-to-chest, pointing ahead; left
    // hand under the fore-end.
    const float handPos[2][3] = {{-0.02f, -0.30f, -0.62f}, {0.10f, -0.32f, -0.30f}};
    for (int i = 0; i < 2; ++i)
    {
        for (HandPose* p : {&out.grip[i], &out.aim[i]})
        {
            *p = HandPose{};
            p->valid = true;
            p->position[0] = handPos[i][0];
            p->position[1] = handPos[i][1] + headY;
            p->position[2] = handPos[i][2];
        }
    }
    // Left grip held, so the two-handed path is what the screenshots show.
    s_controllers = ControllerState{};
    s_controllers.grip[0] = 1.0f;

    // CWR_VR_FAKE_SCRIPT=reload: eject (B), left hand to the chest pouch and
    // grab, to the magazine well under the weapon, let go.
    static const char* script = std::getenv("CWR_VR_FAKE_SCRIPT");
    if (script && std::strcmp(script, "reload") == 0)
    {
        const float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - s_fakeStart).count();
        struct Key
        {
            float time, x, y, z, grip;
            bool b;
        };
        static const Key kKeys[] = {
            {0.0f, -0.02f, -0.30f, -0.62f, 0.0f, false}, // hand on the fore-end, relaxed
            {2.0f, -0.02f, -0.30f, -0.62f, 0.0f, true},  // B: eject
            {2.5f, -0.05f, -0.50f, -0.20f, 0.0f, false}, // hand at the pouch
            {3.5f, -0.05f, -0.50f, -0.20f, 1.0f, false}, // grab
            {4.5f, 0.06f, -0.36f, -0.50f, 1.0f, false},  // at the well: inserts
            {5.5f, -0.02f, -0.30f, -0.62f, 0.0f, false}, // back, let go
        };
        const Key* k = &kKeys[0];
        for (const Key& key : kKeys)
            if (t >= key.time)
                k = &key;
        for (HandPose* p : {&out.grip[0], &out.aim[0]})
        {
            p->position[0] = k->x;
            p->position[1] = k->y + headY;
            p->position[2] = k->z;
        }
        s_controllers.grip[0] = k->grip;
        s_controllers.b = k->b;
    }
    for (int i = 0; i < 2; ++i)
    {
        s_aim[i] = out.aim[i];
        s_grip[i] = out.grip[i];
    }
    out.eyeWidth = out.eyeHeight = static_cast<int>(kFakeEyeSize * s_config.eyeScale);
    out.eyeTexture[0] = out.eyeTexture[1] = 0;
    out.hudTexture = 0;
    out.hudWidth = out.hudHeight = 0;
}

#if defined(_WIN32)

// --- OpenXR state -------------------------------------------------------------

constexpr int kScreenPixelsW = 1920;
constexpr int kScreenPixelsH = 1080;
constexpr int kHudPixelsW = 1920;
constexpr int kHudPixelsH = 1080;

constexpr int64_t kGL_RGBA8 = 0x8058;
constexpr int64_t kGL_SRGB8_ALPHA8 = 0x8C43;

struct Swapchain
{
    XrSwapchain handle = XR_NULL_HANDLE;
    std::vector<XrSwapchainImageOpenGLKHR> images;
    int width = 0;
    int height = 0;
    bool acquired = false;
};

struct State
{
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId systemId = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace localSpace = XR_NULL_HANDLE;
    XrSpace viewSpace = XR_NULL_HANDLE;
    int64_t format = 0;
    Swapchain screen;  // flat virtual screen
    Swapchain eyes[2]; // stereo
    Swapchain hud;     // head-locked HUD
    XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
    XrViewConfigurationView viewConfig[2] = {{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    XrEnvironmentBlendMode blendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    XrSessionState sessionState = XR_SESSION_STATE_UNKNOWN;
    bool running = false;

    // Controller actions (one "gameplay" set, attached once).
    XrActionSet actionSet = XR_NULL_HANDLE;
    XrPath hands[2] = {XR_NULL_PATH, XR_NULL_PATH};
    XrAction move = XR_NULL_HANDLE, turn = XR_NULL_HANDLE;
    XrAction trigger = XR_NULL_HANDLE, grip = XR_NULL_HANDLE, thumbClick = XR_NULL_HANDLE;
    XrAction btnA = XR_NULL_HANDLE, btnB = XR_NULL_HANDLE, btnX = XR_NULL_HANDLE, btnY = XR_NULL_HANDLE;
    XrAction menu = XR_NULL_HANDLE;
    XrAction gripPose = XR_NULL_HANDLE, aimPose = XR_NULL_HANDLE;
    XrAction haptic = XR_NULL_HANDLE;
    XrSpace gripSpace[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
    XrSpace aimSpace[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
    bool frameBegun = false;
    XrTime predictedDisplayTime = 0;

    // Frame timing, summarised to vr.log every kStatsFrames frames.
    static constexpr int kStatsFrames = 600;
    int statsFrames = 0;
    int statsStereo = 0;
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

bool LogSystemAndViews()
{
    XrSystemProperties props{XR_TYPE_SYSTEM_PROPERTIES};
    if (Check(xrGetSystemProperties(s.instance, s.systemId, &props), "xrGetSystemProperties"))
        Log("System: \"%s\" vendor 0x%x, max swapchain %ux%u, max layers %u, tracking orientation=%d position=%d",
            props.systemName, props.vendorId, props.graphicsProperties.maxSwapchainImageWidth,
            props.graphicsProperties.maxSwapchainImageHeight, props.graphicsProperties.maxLayerCount,
            props.trackingProperties.orientationTracking, props.trackingProperties.positionTracking);

    uint32_t viewCount = 0;
    if (!Check(xrEnumerateViewConfigurationViews(s.instance, s.systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                                 0, &viewCount, nullptr),
               "xrEnumerateViewConfigurationViews") ||
        viewCount != 2)
    {
        Log("ERROR: expected 2 stereo views, got %u", viewCount);
        return false;
    }
    xrEnumerateViewConfigurationViews(s.instance, s.systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2,
                                      &viewCount, s.viewConfig);
    for (uint32_t i = 0; i < 2; ++i)
        Log("  view %u: recommended %ux%u (max %ux%u), samples %u", i, s.viewConfig[i].recommendedImageRectWidth,
            s.viewConfig[i].recommendedImageRectHeight, s.viewConfig[i].maxImageRectWidth,
            s.viewConfig[i].maxImageRectHeight, s.viewConfig[i].recommendedSwapchainSampleCount);

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
    return true;
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

bool PickSwapchainFormat()
{
    uint32_t count = 0;
    if (!Check(xrEnumerateSwapchainFormats(s.session, 0, &count, nullptr), "xrEnumerateSwapchainFormats"))
        return false;
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

    // The engine's frames are already gamma-encoded RGBA8.  An sRGB
    // swapchain stores those bytes unchanged (raw blit) and the compositor
    // decodes them correctly; a linear RGBA8 swapchain would show them too
    // bright.
    for (int64_t want : {kGL_SRGB8_ALPHA8, kGL_RGBA8})
        for (auto f : formats)
            if (f == want)
            {
                s.format = f;
                return true;
            }
    s.format = formats.empty() ? 0 : formats[0];
    return s.format != 0;
}

bool CreateSwapchain(Swapchain& sc, int width, int height, const char* what)
{
    XrSwapchainCreateInfo info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
                      XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    info.format = s.format;
    info.sampleCount = 1;
    info.width = width;
    info.height = height;
    info.faceCount = 1;
    info.arraySize = 1;
    info.mipCount = 1;
    if (!Check(xrCreateSwapchain(s.session, &info, &sc.handle), what))
        return false;

    uint32_t count = 0;
    xrEnumerateSwapchainImages(sc.handle, 0, &count, nullptr);
    sc.images.assign(count, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});
    if (!Check(xrEnumerateSwapchainImages(sc.handle, count, &count,
                                          reinterpret_cast<XrSwapchainImageBaseHeader*>(sc.images.data())),
               "xrEnumerateSwapchainImages"))
        return false;
    sc.width = width;
    sc.height = height;
    Log("Swapchain %s: %dx%d format 0x%llx, %u images", what, width, height,
        static_cast<unsigned long long>(s.format), count);
    return true;
}

bool CreateSwapchains()
{
    if (!PickSwapchainFormat() || !CreateSwapchain(s.screen, kScreenPixelsW, kScreenPixelsH, "screen") ||
        !CreateSwapchain(s.hud, kHudPixelsW, kHudPixelsH, "hud"))
        return false;
    for (int i = 0; i < 2; ++i)
    {
        const auto& vc = s.viewConfig[i];
        int w = static_cast<int>(vc.recommendedImageRectWidth * s_config.eyeScale + 0.5f);
        int h = static_cast<int>(vc.recommendedImageRectHeight * s_config.eyeScale + 0.5f);
        if (w > static_cast<int>(vc.maxImageRectWidth))
            w = static_cast<int>(vc.maxImageRectWidth);
        if (h > static_cast<int>(vc.maxImageRectHeight))
            h = static_cast<int>(vc.maxImageRectHeight);
        if (!CreateSwapchain(s.eyes[i], w, h, i == 0 ? "eye 0" : "eye 1"))
            return false;
    }
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

bool CreateSpace(XrReferenceSpaceType type, XrSpace& out, const char* what)
{
    XrReferenceSpaceCreateInfo rsi{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rsi.referenceSpaceType = type;
    rsi.poseInReferenceSpace.orientation.w = 1.0f;
    return Check(xrCreateReferenceSpace(s.session, &rsi, &out), what);
}

XrPath ToPath(const char* str)
{
    XrPath path = XR_NULL_PATH;
    xrStringToPath(s.instance, str, &path);
    return path;
}

bool CreateAction(XrAction& action, const char* name, const char* localized, XrActionType type, bool bothHands)
{
    XrActionCreateInfo info{XR_TYPE_ACTION_CREATE_INFO};
    info.actionType = type;
    std::strncpy(info.actionName, name, XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(info.localizedActionName, localized, XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    if (bothHands)
    {
        info.countSubactionPaths = 2;
        info.subactionPaths = s.hands;
    }
    return Check(xrCreateAction(s.actionSet, &info, &action), name);
}

struct Binding
{
    XrAction* action;
    const char* path;
};

void SuggestBindings(const char* profile, const std::vector<Binding>& bindings)
{
    std::vector<XrActionSuggestedBinding> suggested;
    for (const auto& b : bindings)
        suggested.push_back({*b.action, ToPath(b.path)});
    XrInteractionProfileSuggestedBinding info{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    info.interactionProfile = ToPath(profile);
    info.countSuggestedBindings = static_cast<uint32_t>(suggested.size());
    info.suggestedBindings = suggested.data();
    if (Check(xrSuggestInteractionProfileBindings(s.instance, &info), profile))
        Log("Bindings suggested for %s", profile);
}

// Touch layout; the game sees it as a gamepad (see VRGamepad).
bool CreateActions()
{
    XrActionSetCreateInfo asci{XR_TYPE_ACTION_SET_CREATE_INFO};
    std::strncpy(asci.actionSetName, "gameplay", XR_MAX_ACTION_SET_NAME_SIZE - 1);
    std::strncpy(asci.localizedActionSetName, "Gameplay", XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE - 1);
    if (!Check(xrCreateActionSet(s.instance, &asci, &s.actionSet), "xrCreateActionSet"))
        return false;
    s.hands[0] = ToPath("/user/hand/left");
    s.hands[1] = ToPath("/user/hand/right");

    if (!CreateAction(s.move, "move", "Move", XR_ACTION_TYPE_VECTOR2F_INPUT, false) ||
        !CreateAction(s.turn, "turn", "Turn", XR_ACTION_TYPE_VECTOR2F_INPUT, false) ||
        !CreateAction(s.trigger, "trigger", "Trigger", XR_ACTION_TYPE_FLOAT_INPUT, true) ||
        !CreateAction(s.grip, "grip", "Grip", XR_ACTION_TYPE_FLOAT_INPUT, true) ||
        !CreateAction(s.thumbClick, "thumb_click", "Thumbstick click", XR_ACTION_TYPE_BOOLEAN_INPUT, true) ||
        !CreateAction(s.btnA, "button_a", "A", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
        !CreateAction(s.btnB, "button_b", "B", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
        !CreateAction(s.btnX, "button_x", "X", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
        !CreateAction(s.btnY, "button_y", "Y", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
        !CreateAction(s.menu, "menu", "Menu", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
        !CreateAction(s.gripPose, "grip_pose", "Hand grip pose", XR_ACTION_TYPE_POSE_INPUT, true) ||
        !CreateAction(s.aimPose, "aim_pose", "Hand aim pose", XR_ACTION_TYPE_POSE_INPUT, true) ||
        !CreateAction(s.haptic, "haptic", "Vibration", XR_ACTION_TYPE_VIBRATION_OUTPUT, true))
        return false;

    const std::vector<Binding> common = {
        {&s.move, "/user/hand/left/input/thumbstick"},
        {&s.turn, "/user/hand/right/input/thumbstick"},
        {&s.trigger, "/user/hand/left/input/trigger/value"},
        {&s.trigger, "/user/hand/right/input/trigger/value"},
        {&s.grip, "/user/hand/left/input/squeeze/value"},
        {&s.grip, "/user/hand/right/input/squeeze/value"},
        {&s.thumbClick, "/user/hand/left/input/thumbstick/click"},
        {&s.thumbClick, "/user/hand/right/input/thumbstick/click"},
        {&s.btnA, "/user/hand/right/input/a/click"},
        {&s.btnB, "/user/hand/right/input/b/click"},
        {&s.gripPose, "/user/hand/left/input/grip/pose"},
        {&s.gripPose, "/user/hand/right/input/grip/pose"},
        {&s.aimPose, "/user/hand/left/input/aim/pose"},
        {&s.aimPose, "/user/hand/right/input/aim/pose"},
        {&s.haptic, "/user/hand/left/output/haptic"},
        {&s.haptic, "/user/hand/right/output/haptic"},
    };
    std::vector<Binding> touch = common;
    touch.push_back({&s.btnX, "/user/hand/left/input/x/click"});
    touch.push_back({&s.btnY, "/user/hand/left/input/y/click"});
    touch.push_back({&s.menu, "/user/hand/left/input/menu/click"});
    SuggestBindings("/interaction_profiles/oculus/touch_controller", touch);

    // Index: the left A/B stand in for X/Y; its menu button is the system's.
    std::vector<Binding> index = common;
    index.push_back({&s.btnX, "/user/hand/left/input/a/click"});
    index.push_back({&s.btnY, "/user/hand/left/input/b/click"});
    SuggestBindings("/interaction_profiles/valve/index_controller", index);

    XrSessionActionSetsAttachInfo attach{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    attach.countActionSets = 1;
    attach.actionSets = &s.actionSet;
    if (!Check(xrAttachSessionActionSets(s.session, &attach), "xrAttachSessionActionSets"))
        return false;

    for (int i = 0; i < 2; ++i)
    {
        XrActionSpaceCreateInfo si{XR_TYPE_ACTION_SPACE_CREATE_INFO};
        si.subactionPath = s.hands[i];
        si.poseInActionSpace.orientation.w = 1.0f;
        si.action = s.gripPose;
        if (!Check(xrCreateActionSpace(s.session, &si, &s.gripSpace[i]), "xrCreateActionSpace(grip)"))
            return false;
        si.action = s.aimPose;
        if (!Check(xrCreateActionSpace(s.session, &si, &s.aimSpace[i]), "xrCreateActionSpace(aim)"))
            return false;
    }
    return true;
}

void LocateHand(XrSpace space, HandPose& out)
{
    out = HandPose{};
    if (space == XR_NULL_HANDLE || s.sessionState != XR_SESSION_STATE_FOCUSED)
        return;
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    if (XR_FAILED(xrLocateSpace(space, s.localSpace, s.predictedDisplayTime, &loc)))
        return;
    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT;
    if ((loc.locationFlags & need) != need)
        return;
    out.valid = true;
    out.orientation[0] = loc.pose.orientation.x;
    out.orientation[1] = loc.pose.orientation.y;
    out.orientation[2] = loc.pose.orientation.z;
    out.orientation[3] = loc.pose.orientation.w;
    out.position[0] = loc.pose.position.x;
    out.position[1] = loc.pose.position.y;
    out.position[2] = loc.pose.position.z;
}

void LogInteractionProfiles()
{
    for (int i = 0; i < 2; ++i)
    {
        XrInteractionProfileState st{XR_TYPE_INTERACTION_PROFILE_STATE};
        if (XR_FAILED(xrGetCurrentInteractionProfile(s.session, s.hands[i], &st)))
            continue;
        char name[XR_MAX_PATH_LENGTH] = "none";
        uint32_t len = 0;
        if (st.interactionProfile != XR_NULL_PATH)
            xrPathToString(s.instance, st.interactionProfile, sizeof(name), &len, name);
        Log("Controller %s: %s", i == 0 ? "left" : "right", name);
    }
}

float ActionFloat(XrAction action, XrPath hand)
{
    XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
    gi.action = action;
    gi.subactionPath = hand;
    XrActionStateFloat st{XR_TYPE_ACTION_STATE_FLOAT};
    return XR_SUCCEEDED(xrGetActionStateFloat(s.session, &gi, &st)) && st.isActive ? st.currentState : 0.0f;
}

bool ActionBool(XrAction action, XrPath hand = XR_NULL_PATH)
{
    XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
    gi.action = action;
    gi.subactionPath = hand;
    XrActionStateBoolean st{XR_TYPE_ACTION_STATE_BOOLEAN};
    return XR_SUCCEEDED(xrGetActionStateBoolean(s.session, &gi, &st)) && st.isActive && st.currentState;
}

void ActionVector(XrAction action, float& x, float& y)
{
    XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
    gi.action = action;
    XrActionStateVector2f st{XR_TYPE_ACTION_STATE_VECTOR2F};
    const bool ok = XR_SUCCEEDED(xrGetActionStateVector2f(s.session, &gi, &st)) && st.isActive;
    x = ok ? st.currentState.x : 0.0f;
    y = ok ? st.currentState.y : 0.0f;
}

// Input only flows while the session has focus (not behind the runtime's
// own menus).
void SyncControllers()
{
    s_controllers = ControllerState{};
    if (s.actionSet == XR_NULL_HANDLE || s.sessionState != XR_SESSION_STATE_FOCUSED)
        return;
    XrActiveActionSet active{s.actionSet, XR_NULL_PATH};
    XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    if (XR_FAILED(xrSyncActions(s.session, &sync)))
        return;

    ControllerState& c = s_controllers;
    c.active = true;
    ActionVector(s.move, c.moveX, c.moveY);
    ActionVector(s.turn, c.turnX, c.turnY);
    for (int i = 0; i < 2; ++i)
    {
        c.trigger[i] = ActionFloat(s.trigger, s.hands[i]);
        c.grip[i] = ActionFloat(s.grip, s.hands[i]);
        c.thumbClick[i] = ActionBool(s.thumbClick, s.hands[i]);
    }
    c.a = ActionBool(s.btnA);
    c.b = ActionBool(s.btnB);
    c.x = ActionBool(s.btnX);
    c.y = ActionBool(s.btnY);
    c.menu = ActionBool(s.menu);
}

void DestroyAll()
{
    for (int i = 0; i < 2; ++i)
    {
        if (s.gripSpace[i] != XR_NULL_HANDLE)
            xrDestroySpace(s.gripSpace[i]);
        if (s.aimSpace[i] != XR_NULL_HANDLE)
            xrDestroySpace(s.aimSpace[i]);
    }
    if (s.actionSet != XR_NULL_HANDLE)
        xrDestroyActionSet(s.actionSet); // also destroys its actions
    s_controllers = ControllerState{};
    for (Swapchain* sc : {&s.screen, &s.hud, &s.eyes[0], &s.eyes[1]})
        if (sc->handle != XR_NULL_HANDLE)
            xrDestroySwapchain(sc->handle);
    if (s.viewSpace != XR_NULL_HANDLE)
        xrDestroySpace(s.viewSpace);
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
            case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                LogInteractionProfiles();
                break;
            case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
                Log("Runtime recentred the reference space");
                break;
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

// xrWaitFrame + xrBeginFrame.  Returns the runtime's shouldRender, or false
// when no frame was begun.
bool WaitAndBeginFrame(bool stereo)
{
    PollEvents();
    SyncControllers();
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
    if (stereo)
        s.statsStereo++;
    if (!fs.shouldRender)
        s.statsSkipped++;
    if (s.statsFrames == State::kStatsFrames)
    {
        Log("Frames: %d (stereo %d), avg xrWaitFrame %.2f ms, avg display period %.2f ms, not rendered %d",
            s.statsFrames, s.statsStereo, s.statsWaitMs / s.statsFrames, s.statsPeriodMs / s.statsFrames,
            s.statsSkipped);
        s.statsFrames = s.statsStereo = s.statsSkipped = 0;
        s.statsWaitMs = s.statsPeriodMs = 0.0;
    }
    return fs.shouldRender;
}

unsigned int AcquireImage(Swapchain& sc)
{
    uint32_t index = 0;
    XrSwapchainImageAcquireInfo acq{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    if (!Check(xrAcquireSwapchainImage(sc.handle, &acq, &index), "xrAcquireSwapchainImage"))
        return 0;
    sc.acquired = true;
    XrSwapchainImageWaitInfo wait{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    wait.timeout = XR_INFINITE_DURATION;
    if (!Check(xrWaitSwapchainImage(sc.handle, &wait), "xrWaitSwapchainImage"))
        return 0;
    return sc.images[index].image;
}

// Releases every acquired image; returns false if any release failed.
bool ReleaseImages()
{
    bool ok = true;
    for (Swapchain* sc : {&s.screen, &s.hud, &s.eyes[0], &s.eyes[1]})
    {
        if (!sc->acquired)
            continue;
        XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        ok = Check(xrReleaseSwapchainImage(sc->handle, &rel), "xrReleaseSwapchainImage") && ok;
        sc->acquired = false;
    }
    return ok;
}

void EndFrame(uint32_t layerCount, const XrCompositionLayerBaseHeader* const* layers)
{
    XrFrameEndInfo end{XR_TYPE_FRAME_END_INFO};
    end.displayTime = s.predictedDisplayTime;
    end.environmentBlendMode = s.blendMode;
    end.layerCount = layerCount;
    end.layers = layerCount ? layers : nullptr;
    Check(xrEndFrame(s.session, &end), "xrEndFrame");
    s.frameBegun = false;
}

// Abandon a begun frame: release whatever was acquired, submit no layers.
void AbortFrame()
{
    ReleaseImages();
    EndFrame(0, nullptr);
}

XrSwapchainSubImage SubImage(const Swapchain& sc)
{
    XrSwapchainSubImage sub{};
    sub.swapchain = sc.handle;
    sub.imageRect = {{0, 0}, {sc.width, sc.height}};
    sub.imageArrayIndex = 0;
    return sub;
}

#endif // _WIN32

} // namespace

VRSession& VRSession::Instance()
{
    static VRSession instance;
    return instance;
}

bool VRSession::InitFake()
{
    OpenLog();
    Log("CWR-VR: fake headset (--vr-fake), no OpenXR runtime used");
    LoadConfig();
    s_fake = true;
    s_fakeStart = std::chrono::steady_clock::now();
    Log("Fake eyes: %dx%d, IPD 64 mm, fixed pose", static_cast<int>(kFakeEyeSize * s_config.eyeScale),
        static_cast<int>(kFakeEyeSize * s_config.eyeScale));
    return true;
}

bool VRSession::IsFake() const
{
    return s_fake;
}

const VRConfig& VRSession::Config() const
{
    return s_config;
}

VRConfig& VRSession::MutableConfig()
{
    return s_config;
}

const ControllerState& VRSession::Controllers() const
{
    return s_controllers;
}

const HandPose& VRSession::Aim(int hand) const
{
    return s_aim[hand];
}

const HandPose& VRSession::Grip(int hand) const
{
    return s_grip[hand];
}

void VRSession::Haptic(int hand, float amplitude, float seconds)
{
#if defined(_WIN32)
    if (s_fake || s.haptic == XR_NULL_HANDLE || s.sessionState != XR_SESSION_STATE_FOCUSED)
        return;
    XrHapticVibration vib{XR_TYPE_HAPTIC_VIBRATION};
    vib.amplitude = amplitude;
    vib.duration = static_cast<XrDuration>(seconds * 1.0e9);
    vib.frequency = XR_FREQUENCY_UNSPECIFIED;
    XrHapticActionInfo info{XR_TYPE_HAPTIC_ACTION_INFO};
    info.action = s.haptic;
    info.subactionPath = s.hands[hand];
    xrApplyHapticFeedback(s.session, &info, reinterpret_cast<const XrHapticBaseHeader*>(&vib));
#else
    (void)hand, (void)amplitude, (void)seconds;
#endif
}

void VRSession::LogLine(const char* text)
{
    Log("%s", text);
}

bool VRSession::ReloadConfigIfChanged()
{
    const auto now = std::chrono::steady_clock::now();
    if (now - s_configChecked < std::chrono::seconds(1))
        return false;
    s_configChecked = now;

    const std::string path = ExeDirFile("vr.ini");
    const auto t = ConfigFileTime(path);
    if (t == s_configTime || t == std::filesystem::file_time_type{})
        return false;

    const float eyeScale = s_config.eyeScale;
    LoadConfig();
    if (s_config.eyeScale != eyeScale)
        Log("vr.ini: eye_scale changes take effect on the next start");
    return true;
}

void VRSession::SaveConfig()
{
    const std::string path = ExeDirFile("vr.ini");
    WriteConfig(path, s_config);
    Log("vr.ini: saved from the dev panel");
}

#if defined(_WIN32)

bool VRSession::Init()
{
    if (IsActive())
        return true;

    OpenLog();
    Log("CWR-VR: starting OpenXR");
    LoadConfig();

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
    if (!Check(xrGetSystem(s.instance, &sgi, &s.systemId), "xrGetSystem (is the headset connected?)") ||
        !LogSystemAndViews() || !CheckGLRequirements())
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
    if (!CreateSpace(XR_REFERENCE_SPACE_TYPE_LOCAL, s.localSpace, "xrCreateReferenceSpace(LOCAL)") ||
        !CreateSpace(XR_REFERENCE_SPACE_TYPE_VIEW, s.viewSpace, "xrCreateReferenceSpace(VIEW)") ||
        !CreateSwapchains() || !CreateActions())
    {
        DestroyAll();
        return false;
    }

    Log("Session created; waiting for the runtime to make it READY");
    return true;
}

void VRSession::Shutdown()
{
    if (s.instance != XR_NULL_HANDLE)
    {
        if (s.frameBegun)
            AbortFrame();
        // xrEndSession is only legal in STOPPING; destroying a running
        // session is allowed and the runtime tears it down.
        DestroyAll();
    }
    if (IsActive() || s_fake || s_log)
        Log("Shutdown complete");
    s_fake = false;
    CloseLog();
}

bool VRSession::IsActive() const
{
    return s_fake || s.instance != XR_NULL_HANDLE;
}

bool VRSession::BeginFlatFrame(FlatFrameTarget& out)
{
    if (s_fake)
        return false;
    if (!WaitAndBeginFrame(false))
    {
        if (s.frameBegun)
            EndFrame(0, nullptr);
        return false;
    }

    out.glTexture = AcquireImage(s.screen);
    if (!out.glTexture)
    {
        AbortFrame();
        return false;
    }
    // Hands too, so the controller can point at the flat screen (menus
    // shown without a 3D scene).
    for (int i = 0; i < 2; ++i)
    {
        LocateHand(s.gripSpace[i], s_grip[i]);
        LocateHand(s.aimSpace[i], s_aim[i]);
    }
    out.width = s.screen.width;
    out.height = s.screen.height;
    return true;
}

void VRSession::EndFlatFrame()
{
    if (!s.frameBegun)
        return;
    if (!ReleaseImages())
    {
        EndFrame(0, nullptr);
        return;
    }

    const float w = s_config.screenWidth;
    XrCompositionLayerQuad quad{XR_TYPE_COMPOSITION_LAYER_QUAD};
    quad.space = s.localSpace;
    quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    quad.subImage = SubImage(s.screen);
    quad.pose.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
    quad.pose.position = {0.0f, 0.0f, -s_config.screenDistance};
    quad.size = {w, w * kScreenPixelsH / kScreenPixelsW};
    const XrCompositionLayerBaseHeader* layers[] = {reinterpret_cast<XrCompositionLayerBaseHeader*>(&quad)};
    EndFrame(1, layers);
}

bool VRSession::BeginStereoFrame(StereoFrame& out)
{
    if (s_fake)
    {
        FillFakeFrame(out);
        return true;
    }
    if (!WaitAndBeginFrame(true))
    {
        if (s.frameBegun)
            EndFrame(0, nullptr);
        return false;
    }

    XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO};
    li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    li.displayTime = s.predictedDisplayTime;
    li.space = s.localSpace;
    XrViewState vs{XR_TYPE_VIEW_STATE};
    uint32_t count = 0;
    s.views[0] = s.views[1] = XrView{XR_TYPE_VIEW};
    if (!Check(xrLocateViews(s.session, &li, &vs, 2, &count, s.views), "xrLocateViews") ||
        !(vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT))
    {
        // Tracking lost: nothing sensible to render this frame.
        EndFrame(0, nullptr);
        return false;
    }

    for (int i = 0; i < 2; ++i)
    {
        const XrView& v = s.views[i];
        EyeView& e = out.eyes[i];
        e.orientation[0] = v.pose.orientation.x;
        e.orientation[1] = v.pose.orientation.y;
        e.orientation[2] = v.pose.orientation.z;
        e.orientation[3] = v.pose.orientation.w;
        e.position[0] = v.pose.position.x;
        e.position[1] = v.pose.position.y;
        e.position[2] = v.pose.position.z;
        e.tanLeft = std::tan(v.fov.angleLeft);
        e.tanRight = std::tan(v.fov.angleRight);
        e.tanUp = std::tan(v.fov.angleUp);
        e.tanDown = std::tan(v.fov.angleDown);
        out.eyeTexture[i] = AcquireImage(s.eyes[i]);
        LocateHand(s.gripSpace[i], out.grip[i]);
        LocateHand(s.aimSpace[i], out.aim[i]);
        s_grip[i] = out.grip[i];
        s_aim[i] = out.aim[i];
    }
    out.eyeWidth = s.eyes[0].width;
    out.eyeHeight = s.eyes[0].height;
    out.hudTexture = AcquireImage(s.hud);
    out.hudWidth = s.hud.width;
    out.hudHeight = s.hud.height;

    if (!out.eyeTexture[0] || !out.eyeTexture[1] || !out.hudTexture)
    {
        AbortFrame();
        return false;
    }
    return true;
}

void VRSession::EndStereoFrame(float hudHalfTanX, float hudHalfTanY)
{
    if (s_fake || !s.frameBegun)
        return;
    if (!ReleaseImages())
    {
        EndFrame(0, nullptr);
        return;
    }

    XrCompositionLayerProjectionView pv[2] = {{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},
                                              {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
    for (int i = 0; i < 2; ++i)
    {
        // Must be exactly the pose and FOV the eye was rendered with.
        pv[i].pose = s.views[i].pose;
        pv[i].fov = s.views[i].fov;
        pv[i].subImage = SubImage(s.eyes[i]);
    }
    XrCompositionLayerProjection proj{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    proj.space = s.localSpace;
    proj.viewCount = 2;
    proj.views = pv;

    // Head-locked HUD sized so its angular extent matches the HUD camera's
    // FOV: world-space markers drawn by the HUD then line up with the scene.
    const float d = s_config.hudDistance;
    XrCompositionLayerQuad hud{XR_TYPE_COMPOSITION_LAYER_QUAD};
    hud.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
                     XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
    hud.space = s.viewSpace;
    hud.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    hud.subImage = SubImage(s.hud);
    hud.pose.orientation = {0.0f, 0.0f, 0.0f, 1.0f};
    hud.pose.position = {0.0f, 0.0f, -d};
    hud.size = {2.0f * d * hudHalfTanX, 2.0f * d * hudHalfTanY};

    const XrCompositionLayerBaseHeader* layers[] = {reinterpret_cast<XrCompositionLayerBaseHeader*>(&proj),
                                                    reinterpret_cast<XrCompositionLayerBaseHeader*>(&hud)};
    EndFrame(2, layers);
}

#else // !_WIN32

bool VRSession::Init()
{
    OpenLog();
    Log("CWR-VR: OpenXR is only implemented on Windows so far. Continuing flat (--vr-fake still works).");
    CloseLog();
    return false;
}

void VRSession::Shutdown()
{
    if (s_fake)
        Log("Shutdown complete");
    s_fake = false;
    CloseLog();
}

bool VRSession::IsActive() const
{
    return s_fake;
}

bool VRSession::BeginFlatFrame(FlatFrameTarget&)
{
    return false;
}

void VRSession::EndFlatFrame() {}

bool VRSession::BeginStereoFrame(StereoFrame& out)
{
    if (!s_fake)
        return false;
    FillFakeFrame(out);
    return true;
}

void VRSession::EndStereoFrame(float, float) {}

#endif // _WIN32

} // namespace PoseidonVR
