#define NOMINMAX

#include "vr_openxr.h"

#ifdef ENABLE_DX11

#include <vector>
#include <string>
#include <cstring>
#include <cmath>

#include <d3d11.h>
#include <wrl/client.h>
using Microsoft::WRL::ComPtr;

#define XR_USE_GRAPHICS_API_D3D11
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <spdlog/spdlog.h>

#include "public/bridge/consolevariablebridge.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

// D3D11 device accessors (defined in gfx_direct3d11.cpp)
extern void* gfx_d3d11_get_device();
extern void* gfx_d3d11_get_context();
extern void gfx_d3d11_set_render_target_height(uint32_t height);

// --------------------------------------------------------------------------
// Internal state
// --------------------------------------------------------------------------

static struct {
    // OpenXR handles
    XrInstance instance;
    XrSystemId system_id;
    XrSession session;
    XrSpace local_space;
    XrSessionState session_state;
    bool session_running;

    // View configuration
    uint32_t view_count;
    XrView views[2];
    XrViewConfigurationView config_views[2];

    // Swapchains (one per eye)
    struct EyeSwapchain {
        XrSwapchain handle;
        int64_t format;
        uint32_t width, height;
        std::vector<XrSwapchainImageD3D11KHR> images;
        std::vector<ComPtr<ID3D11RenderTargetView>> rtvs;
        std::vector<ComPtr<ID3D11DepthStencilView>> dsvs;
        std::vector<ComPtr<ID3D11Texture2D>> depth_textures;
    } eye_swapchains[2];

    // Per-frame
    XrFrameState frame_state;
    bool frame_began;
    int current_eye;
    uint32_t refresh_rate;  // Cached headset refresh in Hz, derived from predictedDisplayPeriod
    uint32_t current_image_index[2]; // Acquired swapchain image index per eye

    // Cached per-frame matrices (row-major, row-vector convention)
    float projection[2][4][4];
    float view[2][4][4];

    // Configuration
    float world_scale;       // N64 units per meter
    float near_clip;         // In game units
    float far_clip;          // In game units
    float resolution_scale;  // Multiplier on the runtime's recommended per-eye resolution

    // HUD overlay
    XrSpace view_space;
    struct EyeSwapchain hud_swapchain;
    uint32_t hud_image_index;
    void* hud_commands;
    bool rendering_hud;

    // D3D11 cached pointers
    ID3D11Device* d3d_device;
    ID3D11DeviceContext* d3d_context;

    bool initialized;
} xr = {};

// --------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------

static bool xr_check(XrResult result, const char* msg) {
    if (XR_SUCCEEDED(result)) return true;
    if (xr.instance != XR_NULL_HANDLE) {
        char buf[XR_MAX_RESULT_STRING_SIZE];
        xrResultToString(xr.instance, result, buf);
        spdlog::error("[VR] {} failed: {}", msg, buf);
    } else {
        spdlog::error("[VR] {} failed: XrResult {}", msg, static_cast<int>(result));
    }
    return false;
}

// Build an asymmetric projection matrix from XrFovf.
// Build asymmetric projection from XrFovf.
// Output is row-major for the engine's row-vector convention (clip = v * P).
// This is the TRANSPOSE of the standard column-vector OpenGL projection.
static void build_projection_matrix(const XrFovf& fov, float near_z, float far_z, float out[4][4]) {
    float left = tanf(fov.angleLeft);
    float right = tanf(fov.angleRight);
    float up = tanf(fov.angleUp);
    float down = tanf(fov.angleDown);

    float width = right - left;
    float height = up - down;
    float depth = far_z - near_z;

    memset(out, 0, sizeof(float) * 16);

    // Row-vector convention (transposed from column-vector):
    out[0][0] = 2.0f / width;
    out[1][1] = 2.0f / height;
    out[2][0] = (right + left) / width;
    out[2][1] = (up + down) / height;
    out[2][2] = -(far_z + near_z) / depth;
    out[2][3] = -1.0f;
    out[3][2] = -(2.0f * far_z * near_z) / depth;
}

// Convert XrPosef to a view matrix (inverse of the pose).
// Applies world_scale to translation.
// Output is row-major for row-vector convention.
static void pose_to_view_matrix(const XrPosef& pose, float world_scale, float out[4][4]) {
    glm::quat q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    glm::mat4 rotation = glm::mat4_cast(q);
    glm::vec3 pos(pose.position.x * world_scale, pose.position.y * world_scale, pose.position.z * world_scale);

    glm::mat4 transform = glm::translate(glm::mat4(1.0f), pos) * rotation;
    glm::mat4 view = glm::inverse(transform);

    // GLM is column-major and column-vector (v' = M * v).
    // Engine uses row-vector (v' = v * M^T), so we need the mathematical transpose.
    // GLM: view[col][row], so view[r][c] = element(c, r) = transposed element(r, c).
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            out[r][c] = view[r][c];
}

// --------------------------------------------------------------------------
// OpenXR session state event handling
// --------------------------------------------------------------------------

static void handle_session_state_change(XrSessionState new_state) {
    xr.session_state = new_state;

    switch (new_state) {
        case XR_SESSION_STATE_READY: {
            XrSessionBeginInfo begin_info = { XR_TYPE_SESSION_BEGIN_INFO };
            begin_info.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            if (xr_check(xrBeginSession(xr.session, &begin_info), "xrBeginSession")) {
                xr.session_running = true;
                spdlog::info("[VR] Session started");
            }
            break;
        }
        case XR_SESSION_STATE_STOPPING: {
            xr.session_running = false;
            xr_check(xrEndSession(xr.session), "xrEndSession");
            spdlog::info("[VR] Session stopped");
            break;
        }
        case XR_SESSION_STATE_LOSS_PENDING:
        case XR_SESSION_STATE_EXITING:
            xr.session_running = false;
            xr.initialized = false;
            spdlog::warn("[VR] Session lost or exiting");
            break;
        default:
            break;
    }
}

static void poll_events() {
    XrEventDataBuffer event = { XR_TYPE_EVENT_DATA_BUFFER };
    while (xrPollEvent(xr.instance, &event) == XR_SUCCESS) {
        if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            auto* state_event = reinterpret_cast<XrEventDataSessionStateChanged*>(&event);
            handle_session_state_change(state_event->state);
        }
        event = { XR_TYPE_EVENT_DATA_BUFFER };
    }
}

// --------------------------------------------------------------------------
// Lifecycle
// --------------------------------------------------------------------------

bool vr_init() {
    // Get D3D11 device
    xr.d3d_device = static_cast<ID3D11Device*>(gfx_d3d11_get_device());
    xr.d3d_context = static_cast<ID3D11DeviceContext*>(gfx_d3d11_get_context());
    if (!xr.d3d_device || !xr.d3d_context) {
        spdlog::error("[VR] D3D11 device not available");
        return false;
    }

    // Default configuration
    xr.world_scale = 35.0f;
    xr.near_clip = 10.0f;
    xr.far_clip = 30000.0f;

    // Per-eye resolution multiplier. Runtimes (esp. SteamVR) often bake a supersampling
    // factor into the "recommended" size, so each eye can be 1.4-2x the panel resolution.
    // This is the main GPU-cost lever in VR; drop below 1.0 to trade sharpness for framerate.
    // Tunable via the gVrResolutionScale CVar (takes effect on next vr_init).
    xr.resolution_scale = CVarGetFloat("gVrResolutionScale", 1.0f);
    if (xr.resolution_scale < 0.1f) xr.resolution_scale = 0.1f;
    if (xr.resolution_scale > 2.0f) xr.resolution_scale = 2.0f;

    // Sane default until the first frame is located and we can read the true display period.
    xr.refresh_rate = 90;

    // --- Create Instance ---
    const char* extensions[] = { XR_KHR_D3D11_ENABLE_EXTENSION_NAME };

    XrInstanceCreateInfo instance_ci = { XR_TYPE_INSTANCE_CREATE_INFO };
    strcpy(instance_ci.applicationInfo.applicationName, "Ship of Harkinian VR");
    instance_ci.applicationInfo.applicationVersion = 1;
    strcpy(instance_ci.applicationInfo.engineName, "libultraship");
    instance_ci.applicationInfo.engineVersion = 1;
    instance_ci.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    instance_ci.enabledExtensionCount = 1;
    instance_ci.enabledExtensionNames = extensions;

    if (!xr_check(xrCreateInstance(&instance_ci, &xr.instance), "xrCreateInstance")) {
        spdlog::error("[VR] Failed to create OpenXR instance. Make sure SteamVR is running.");
        return false;
    }

    // --- Get System ---
    XrSystemGetInfo system_info = { XR_TYPE_SYSTEM_GET_INFO };
    system_info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    if (!xr_check(xrGetSystem(xr.instance, &system_info, &xr.system_id), "xrGetSystem")) {
        xrDestroyInstance(xr.instance);
        xr.instance = XR_NULL_HANDLE;
        return false;
    }

    // --- Check D3D11 graphics requirements ---
    PFN_xrGetD3D11GraphicsRequirementsKHR xrGetD3D11GraphicsRequirementsKHR = nullptr;
    xrGetInstanceProcAddr(xr.instance, "xrGetD3D11GraphicsRequirementsKHR",
                          reinterpret_cast<PFN_xrVoidFunction*>(&xrGetD3D11GraphicsRequirementsKHR));

    XrGraphicsRequirementsD3D11KHR gfx_requirements = { XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR };
    if (xrGetD3D11GraphicsRequirementsKHR) {
        xrGetD3D11GraphicsRequirementsKHR(xr.instance, xr.system_id, &gfx_requirements);
    }

    // --- Create Session ---
    XrGraphicsBindingD3D11KHR d3d_binding = { XR_TYPE_GRAPHICS_BINDING_D3D11_KHR };
    d3d_binding.device = xr.d3d_device;

    XrSessionCreateInfo session_ci = { XR_TYPE_SESSION_CREATE_INFO };
    session_ci.next = &d3d_binding;
    session_ci.systemId = xr.system_id;
    if (!xr_check(xrCreateSession(xr.instance, &session_ci, &xr.session), "xrCreateSession")) {
        xrDestroyInstance(xr.instance);
        xr.instance = XR_NULL_HANDLE;
        return false;
    }

    // --- Create Reference Space ---
    XrReferenceSpaceCreateInfo space_ci = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    space_ci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    space_ci.poseInReferenceSpace = { { 0, 0, 0, 1 }, { 0, 0, 0 } }; // Identity
    if (!xr_check(xrCreateReferenceSpace(xr.session, &space_ci, &xr.local_space), "xrCreateReferenceSpace")) {
        xrDestroySession(xr.session);
        xrDestroyInstance(xr.instance);
        return false;
    }

    // --- Enumerate View Configuration ---
    uint32_t view_count = 0;
    xrEnumerateViewConfigurationViews(xr.instance, xr.system_id,
                                      XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &view_count, nullptr);
    if (view_count != 2) {
        spdlog::error("[VR] Expected 2 views for stereo, got {}", view_count);
        vr_shutdown();
        return false;
    }
    xr.view_count = 2;
    xr.config_views[0] = { XR_TYPE_VIEW_CONFIGURATION_VIEW };
    xr.config_views[1] = { XR_TYPE_VIEW_CONFIGURATION_VIEW };
    xrEnumerateViewConfigurationViews(xr.instance, xr.system_id,
                                      XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                      2, &view_count, xr.config_views);

    spdlog::info("[VR] Recommended render resolution: {}x{} per eye",
                 xr.config_views[0].recommendedImageRectWidth,
                 xr.config_views[0].recommendedImageRectHeight);

    // --- Enumerate Swapchain Formats ---
    uint32_t format_count = 0;
    xrEnumerateSwapchainFormats(xr.session, 0, &format_count, nullptr);
    std::vector<int64_t> formats(format_count);
    xrEnumerateSwapchainFormats(xr.session, format_count, &format_count, formats.data());

    // Prefer UNORM to match the game's rendering pipeline (which outputs gamma-space colors).
    // SRGB would double-encode gamma, causing a washed-out/hazy appearance.
    // Fall back to SRGB if UNORM is not supported by the runtime.
    int64_t chosen_format = formats[0]; // fallback to first supported
    for (int64_t fmt : formats) {
        if (fmt == DXGI_FORMAT_R8G8B8A8_UNORM) {
            chosen_format = fmt;
            break;
        }
    }
    if (chosen_format != DXGI_FORMAT_R8G8B8A8_UNORM) {
        // UNORM not available — pick SRGB as next best, accept the gamma mismatch for now
        for (int64_t fmt : formats) {
            if (fmt == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) {
                chosen_format = fmt;
                break;
            }
        }
    }
    spdlog::info("[VR] Swapchain format: {} (UNORM={}, SRGB={})",
                 chosen_format, (int)DXGI_FORMAT_R8G8B8A8_UNORM, (int)DXGI_FORMAT_R8G8B8A8_UNORM_SRGB);

    // --- Create Swapchains (one per eye) ---
    for (uint32_t eye = 0; eye < 2; eye++) {
        auto& sc = xr.eye_swapchains[eye];

        // Apply the resolution multiplier, then clamp to what the runtime allows.
        uint32_t scaled_w = (uint32_t)lroundf(xr.config_views[eye].recommendedImageRectWidth * xr.resolution_scale);
        uint32_t scaled_h = (uint32_t)lroundf(xr.config_views[eye].recommendedImageRectHeight * xr.resolution_scale);
        if (scaled_w < 1) scaled_w = 1;
        if (scaled_h < 1) scaled_h = 1;
        if (scaled_w > xr.config_views[eye].maxImageRectWidth) scaled_w = xr.config_views[eye].maxImageRectWidth;
        if (scaled_h > xr.config_views[eye].maxImageRectHeight) scaled_h = xr.config_views[eye].maxImageRectHeight;

        sc.width = scaled_w;
        sc.height = scaled_h;
        sc.format = chosen_format;

        spdlog::info("[VR] Eye {} render resolution: {}x{} (recommended {}x{}, scale {:.2f})", eye, sc.width, sc.height,
                     xr.config_views[eye].recommendedImageRectWidth, xr.config_views[eye].recommendedImageRectHeight,
                     xr.resolution_scale);

        XrSwapchainCreateInfo swapchain_ci = { XR_TYPE_SWAPCHAIN_CREATE_INFO };
        swapchain_ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        swapchain_ci.format = chosen_format;
        swapchain_ci.sampleCount = 1;
        swapchain_ci.width = sc.width;
        swapchain_ci.height = sc.height;
        swapchain_ci.faceCount = 1;
        swapchain_ci.arraySize = 1;
        swapchain_ci.mipCount = 1;

        if (!xr_check(xrCreateSwapchain(xr.session, &swapchain_ci, &sc.handle), "xrCreateSwapchain")) {
            vr_shutdown();
            return false;
        }

        // Enumerate swapchain images
        uint32_t image_count = 0;
        xrEnumerateSwapchainImages(sc.handle, 0, &image_count, nullptr);
        sc.images.resize(image_count, { XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR });
        xrEnumerateSwapchainImages(sc.handle, image_count, &image_count,
                                   reinterpret_cast<XrSwapchainImageBaseHeader*>(sc.images.data()));

        // Create RTVs and depth resources for each swapchain image
        sc.rtvs.resize(image_count);
        sc.dsvs.resize(image_count);
        sc.depth_textures.resize(image_count);

        for (uint32_t i = 0; i < image_count; i++) {
            // RTV
            D3D11_RENDER_TARGET_VIEW_DESC rtv_desc = {};
            rtv_desc.Format = static_cast<DXGI_FORMAT>(chosen_format);
            rtv_desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
            rtv_desc.Texture2D.MipSlice = 0;
            HRESULT hr = xr.d3d_device->CreateRenderTargetView(
                sc.images[i].texture, &rtv_desc, sc.rtvs[i].GetAddressOf());
            if (FAILED(hr)) {
                spdlog::error("[VR] Failed to create RTV for eye {} image {}", eye, i);
                vr_shutdown();
                return false;
            }

            // Depth texture
            D3D11_TEXTURE2D_DESC depth_desc = {};
            depth_desc.Width = sc.width;
            depth_desc.Height = sc.height;
            depth_desc.MipLevels = 1;
            depth_desc.ArraySize = 1;
            depth_desc.Format = DXGI_FORMAT_D32_FLOAT;
            depth_desc.SampleDesc.Count = 1;
            depth_desc.Usage = D3D11_USAGE_DEFAULT;
            depth_desc.BindFlags = D3D11_BIND_DEPTH_STENCIL;

            hr = xr.d3d_device->CreateTexture2D(&depth_desc, nullptr, sc.depth_textures[i].GetAddressOf());
            if (FAILED(hr)) {
                spdlog::error("[VR] Failed to create depth texture for eye {} image {}", eye, i);
                vr_shutdown();
                return false;
            }

            // DSV
            D3D11_DEPTH_STENCIL_VIEW_DESC dsv_desc = {};
            dsv_desc.Format = DXGI_FORMAT_D32_FLOAT;
            dsv_desc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
            dsv_desc.Texture2D.MipSlice = 0;

            hr = xr.d3d_device->CreateDepthStencilView(
                sc.depth_textures[i].Get(), &dsv_desc, sc.dsvs[i].GetAddressOf());
            if (FAILED(hr)) {
                spdlog::error("[VR] Failed to create DSV for eye {} image {}", eye, i);
                vr_shutdown();
                return false;
            }
        }

        spdlog::info("[VR] Eye {} swapchain: {}x{}, {} images", eye, sc.width, sc.height, image_count);
    }

    // --- Create VIEW reference space (head-locked, for HUD overlay) ---
    XrReferenceSpaceCreateInfo view_space_ci = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    view_space_ci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    view_space_ci.poseInReferenceSpace = { { 0, 0, 0, 1 }, { 0, 0, 0 } };
    if (!xr_check(xrCreateReferenceSpace(xr.session, &view_space_ci, &xr.view_space), "xrCreateReferenceSpace (VIEW)")) {
        vr_shutdown();
        return false;
    }

    // --- Create HUD swapchain (1024x768, 4:3) ---
    {
        auto& sc = xr.hud_swapchain;
        sc.width = 1024;
        sc.height = 768;
        sc.format = chosen_format;

        XrSwapchainCreateInfo swapchain_ci = { XR_TYPE_SWAPCHAIN_CREATE_INFO };
        swapchain_ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        swapchain_ci.format = chosen_format;
        swapchain_ci.sampleCount = 1;
        swapchain_ci.width = sc.width;
        swapchain_ci.height = sc.height;
        swapchain_ci.faceCount = 1;
        swapchain_ci.arraySize = 1;
        swapchain_ci.mipCount = 1;

        if (!xr_check(xrCreateSwapchain(xr.session, &swapchain_ci, &sc.handle), "xrCreateSwapchain (HUD)")) {
            vr_shutdown();
            return false;
        }

        uint32_t image_count = 0;
        xrEnumerateSwapchainImages(sc.handle, 0, &image_count, nullptr);
        sc.images.resize(image_count, { XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR });
        xrEnumerateSwapchainImages(sc.handle, image_count, &image_count,
                                   reinterpret_cast<XrSwapchainImageBaseHeader*>(sc.images.data()));

        sc.rtvs.resize(image_count);
        sc.dsvs.resize(image_count);
        sc.depth_textures.resize(image_count);

        for (uint32_t i = 0; i < image_count; i++) {
            D3D11_RENDER_TARGET_VIEW_DESC rtv_desc = {};
            rtv_desc.Format = static_cast<DXGI_FORMAT>(chosen_format);
            rtv_desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
            xr.d3d_device->CreateRenderTargetView(sc.images[i].texture, &rtv_desc, sc.rtvs[i].GetAddressOf());

            D3D11_TEXTURE2D_DESC depth_desc = {};
            depth_desc.Width = sc.width;
            depth_desc.Height = sc.height;
            depth_desc.MipLevels = 1;
            depth_desc.ArraySize = 1;
            depth_desc.Format = DXGI_FORMAT_D32_FLOAT;
            depth_desc.SampleDesc.Count = 1;
            depth_desc.Usage = D3D11_USAGE_DEFAULT;
            depth_desc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
            xr.d3d_device->CreateTexture2D(&depth_desc, nullptr, sc.depth_textures[i].GetAddressOf());

            D3D11_DEPTH_STENCIL_VIEW_DESC dsv_desc = {};
            dsv_desc.Format = DXGI_FORMAT_D32_FLOAT;
            dsv_desc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
            xr.d3d_device->CreateDepthStencilView(sc.depth_textures[i].Get(), &dsv_desc, sc.dsvs[i].GetAddressOf());
        }
        spdlog::info("[VR] HUD swapchain: {}x{}, {} images", sc.width, sc.height, image_count);
    }

    // Initialize views
    xr.views[0] = { XR_TYPE_VIEW };
    xr.views[1] = { XR_TYPE_VIEW };

    xr.initialized = true;
    spdlog::info("[VR] OpenXR initialized successfully");
    return true;
}

void vr_shutdown() {
    xr.initialized = false;
    xr.session_running = false;

    for (uint32_t eye = 0; eye < 2; eye++) {
        auto& sc = xr.eye_swapchains[eye];
        sc.rtvs.clear();
        sc.dsvs.clear();
        sc.depth_textures.clear();
        sc.images.clear();
        if (sc.handle != XR_NULL_HANDLE) {
            xrDestroySwapchain(sc.handle);
            sc.handle = XR_NULL_HANDLE;
        }
    }

    {
        auto& sc = xr.hud_swapchain;
        sc.rtvs.clear(); sc.dsvs.clear(); sc.depth_textures.clear(); sc.images.clear();
        if (sc.handle != XR_NULL_HANDLE) { xrDestroySwapchain(sc.handle); sc.handle = XR_NULL_HANDLE; }
    }
    if (xr.view_space != XR_NULL_HANDLE) {
        xrDestroySpace(xr.view_space);
        xr.view_space = XR_NULL_HANDLE;
    }
    if (xr.local_space != XR_NULL_HANDLE) {
        xrDestroySpace(xr.local_space);
        xr.local_space = XR_NULL_HANDLE;
    }
    if (xr.session != XR_NULL_HANDLE) {
        xrDestroySession(xr.session);
        xr.session = XR_NULL_HANDLE;
    }
    if (xr.instance != XR_NULL_HANDLE) {
        xrDestroyInstance(xr.instance);
        xr.instance = XR_NULL_HANDLE;
    }

    spdlog::info("[VR] OpenXR shut down");
}

// --------------------------------------------------------------------------
// Per-frame
// --------------------------------------------------------------------------

bool vr_begin_frame() {
    if (!xr.initialized) return false;

    poll_events();

    if (!xr.session_running) return false;

    // Wait for the runtime to signal it's ready for a new frame
    xr.frame_state = { XR_TYPE_FRAME_STATE };
    XrFrameWaitInfo wait_info = { XR_TYPE_FRAME_WAIT_INFO };
    if (!xr_check(xrWaitFrame(xr.session, &wait_info, &xr.frame_state), "xrWaitFrame")) {
        return false;
    }

    // Derive the headset refresh rate from the nominal display period (nanoseconds). This paces
    // the game's fixed-timestep logic via the interpolation system (see GetInterpolationFPS).
    if (xr.frame_state.predictedDisplayPeriod > 0) {
        uint32_t hz = (uint32_t)(1.0e9 / (double)xr.frame_state.predictedDisplayPeriod + 0.5);
        if (hz >= 30 && hz <= 1000) {
            xr.refresh_rate = hz;
        }
    }

    XrFrameBeginInfo begin_info = { XR_TYPE_FRAME_BEGIN_INFO };
    if (!xr_check(xrBeginFrame(xr.session, &begin_info), "xrBeginFrame")) {
        return false;
    }
    xr.frame_began = true;

    if (!xr.frame_state.shouldRender) {
        return false;
    }

    // Locate views (get per-eye pose and FOV)
    XrViewState view_state = { XR_TYPE_VIEW_STATE };
    XrViewLocateInfo view_locate_info = { XR_TYPE_VIEW_LOCATE_INFO };
    view_locate_info.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    view_locate_info.displayTime = xr.frame_state.predictedDisplayTime;
    view_locate_info.space = xr.local_space;

    uint32_t view_count = 2;
    XrResult result = xrLocateViews(xr.session, &view_locate_info, &view_state, 2, &view_count, xr.views);
    if (!XR_SUCCEEDED(result)) {
        spdlog::warn("[VR] xrLocateViews failed");
        return false;
    }

    // Build matrices for each eye
    for (int eye = 0; eye < 2; eye++) {
        build_projection_matrix(xr.views[eye].fov, xr.near_clip, xr.far_clip, xr.projection[eye]);
        pose_to_view_matrix(xr.views[eye].pose, xr.world_scale, xr.view[eye]);
    }

    return true;
}

void vr_end_frame() {
    if (!xr.frame_began) return;
    xr.frame_began = false;

    XrCompositionLayerProjectionView projection_views[2] = {};
    for (int eye = 0; eye < 2; eye++) {
        projection_views[eye] = { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW };
        projection_views[eye].pose = xr.views[eye].pose;
        projection_views[eye].fov = xr.views[eye].fov;
        projection_views[eye].subImage.swapchain = xr.eye_swapchains[eye].handle;
        projection_views[eye].subImage.imageRect.offset = { 0, 0 };
        projection_views[eye].subImage.imageRect.extent = {
            static_cast<int32_t>(xr.eye_swapchains[eye].width),
            static_cast<int32_t>(xr.eye_swapchains[eye].height)
        };
        projection_views[eye].subImage.imageArrayIndex = 0;
    }

    XrCompositionLayerProjection projection_layer = { XR_TYPE_COMPOSITION_LAYER_PROJECTION };
    projection_layer.space = xr.local_space;
    projection_layer.viewCount = 2;
    projection_layer.views = projection_views;

    // HUD quad layer (head-locked, alpha-blended)
    XrCompositionLayerQuad hud_layer = { XR_TYPE_COMPOSITION_LAYER_QUAD };
    hud_layer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
    hud_layer.space = xr.view_space;
    hud_layer.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    hud_layer.subImage.swapchain = xr.hud_swapchain.handle;
    hud_layer.subImage.imageRect.offset = { 0, 0 };
    hud_layer.subImage.imageRect.extent = {
        static_cast<int32_t>(xr.hud_swapchain.width),
        static_cast<int32_t>(xr.hud_swapchain.height)
    };
    hud_layer.subImage.imageArrayIndex = 0;
    hud_layer.pose = { { 0, 0, 0, 1 }, { 0, 0, -2.0f } };
    hud_layer.size = { 1.5f, 1.125f };

    const XrCompositionLayerBaseHeader* layers[] = {
        reinterpret_cast<const XrCompositionLayerBaseHeader*>(&projection_layer),
        reinterpret_cast<const XrCompositionLayerBaseHeader*>(&hud_layer)
    };

    XrFrameEndInfo end_info = { XR_TYPE_FRAME_END_INFO };
    end_info.displayTime = xr.frame_state.predictedDisplayTime;
    end_info.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;

    if (xr.frame_state.shouldRender) {
        end_info.layerCount = 2;
        end_info.layers = layers;
    } else {
        end_info.layerCount = 0;
        end_info.layers = nullptr;
    }

    xr_check(xrEndFrame(xr.session, &end_info), "xrEndFrame");
}

// --------------------------------------------------------------------------
// Per-eye
// --------------------------------------------------------------------------

void vr_begin_eye(int eye) {
    if (!xr.initialized) return;
    xr.current_eye = eye;

    auto& sc = xr.eye_swapchains[eye];

    // Acquire swapchain image
    XrSwapchainImageAcquireInfo acquire_info = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    uint32_t image_index = 0;
    xr_check(xrAcquireSwapchainImage(sc.handle, &acquire_info, &image_index), "xrAcquireSwapchainImage");
    xr.current_image_index[eye] = image_index;

    // Wait for it to be ready
    XrSwapchainImageWaitInfo wait_info = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    wait_info.timeout = XR_INFINITE_DURATION;
    xr_check(xrWaitSwapchainImage(sc.handle, &wait_info), "xrWaitSwapchainImage");

    // Bind render target
    ID3D11RenderTargetView* rtv = sc.rtvs[image_index].Get();
    ID3D11DepthStencilView* dsv = sc.dsvs[image_index].Get();
    xr.d3d_context->OMSetRenderTargets(1, &rtv, dsv);

    // Clear
    float clear_color[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    xr.d3d_context->ClearRenderTargetView(rtv, clear_color);
    xr.d3d_context->ClearDepthStencilView(dsv, D3D11_CLEAR_DEPTH, 1.0f, 0);

    // Set viewport
    D3D11_VIEWPORT viewport = {};
    viewport.TopLeftX = 0;
    viewport.TopLeftY = 0;
    viewport.Width = static_cast<float>(sc.width);
    viewport.Height = static_cast<float>(sc.height);
    viewport.MinDepth = 0.0f;
    viewport.MaxDepth = 1.0f;
    xr.d3d_context->RSSetViewports(1, &viewport);

    // Tell D3D11 backend the render target height so viewport Y-flip works correctly
    gfx_d3d11_set_render_target_height(sc.height);
}

void vr_end_eye(int eye) {
    if (!xr.initialized) return;

    auto& sc = xr.eye_swapchains[eye];
    XrSwapchainImageReleaseInfo release_info = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    xr_check(xrReleaseSwapchainImage(sc.handle, &release_info), "xrReleaseSwapchainImage");
}

// --------------------------------------------------------------------------
// Matrix queries
// --------------------------------------------------------------------------

void vr_get_projection_matrix(int eye, float out[4][4]) {
    memcpy(out, xr.projection[eye], sizeof(float) * 16);
}

void vr_get_view_matrix(int eye, float out[4][4]) {
    memcpy(out, xr.view[eye], sizeof(float) * 16);
}

// --------------------------------------------------------------------------
// State queries
// --------------------------------------------------------------------------

bool vr_is_initialized() {
    return xr.initialized;
}

int vr_get_current_eye() {
    return xr.current_eye;
}

void vr_get_recommended_resolution(uint32_t* width, uint32_t* height) {
    if (xr.initialized) {
        // Return the actual (scaled) swapchain size, not the raw recommendation, so the
        // engine's render dimensions match the viewport bound in vr_begin_eye().
        *width = xr.eye_swapchains[0].width;
        *height = xr.eye_swapchains[0].height;
    }
}

uint32_t vr_get_refresh_rate() {
    return xr.refresh_rate ? xr.refresh_rate : 90;
}

float vr_get_world_scale() {
    return xr.world_scale;
}

void vr_set_world_scale(float units_per_meter) {
    xr.world_scale = units_per_meter;
}

void vr_rebind_current_eye_target() {
    if (!xr.initialized || !xr.frame_began) return;

    auto& sc = xr.eye_swapchains[xr.current_eye];
    uint32_t idx = xr.current_image_index[xr.current_eye];

    ID3D11RenderTargetView* rtv = sc.rtvs[idx].Get();
    ID3D11DepthStencilView* dsv = sc.dsvs[idx].Get();
    xr.d3d_context->OMSetRenderTargets(1, &rtv, dsv);
    gfx_d3d11_set_render_target_height(sc.height);
}

// --------------------------------------------------------------------------
// HUD overlay
// --------------------------------------------------------------------------

void vr_set_hud_commands(void* commands) { xr.hud_commands = commands; }
void* vr_get_hud_commands() { return xr.hud_commands; }

void vr_begin_hud() {
    if (!xr.initialized) return;
    xr.rendering_hud = true;

    auto& sc = xr.hud_swapchain;
    XrSwapchainImageAcquireInfo acquire_info = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    uint32_t image_index = 0;
    xr_check(xrAcquireSwapchainImage(sc.handle, &acquire_info, &image_index), "xrAcquireSwapchainImage (HUD)");
    xr.hud_image_index = image_index;

    XrSwapchainImageWaitInfo wait_info = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    wait_info.timeout = XR_INFINITE_DURATION;
    xr_check(xrWaitSwapchainImage(sc.handle, &wait_info), "xrWaitSwapchainImage (HUD)");

    ID3D11RenderTargetView* rtv = sc.rtvs[image_index].Get();
    ID3D11DepthStencilView* dsv = sc.dsvs[image_index].Get();
    xr.d3d_context->OMSetRenderTargets(1, &rtv, dsv);

    float clear_color[] = { 0.0f, 0.0f, 0.0f, 0.0f }; // Transparent
    xr.d3d_context->ClearRenderTargetView(rtv, clear_color);
    xr.d3d_context->ClearDepthStencilView(dsv, D3D11_CLEAR_DEPTH, 1.0f, 0);

    D3D11_VIEWPORT viewport = {};
    viewport.Width = static_cast<float>(sc.width);
    viewport.Height = static_cast<float>(sc.height);
    viewport.MinDepth = 0.0f;
    viewport.MaxDepth = 1.0f;
    xr.d3d_context->RSSetViewports(1, &viewport);

    gfx_d3d11_set_render_target_height(sc.height);
}

void vr_end_hud() {
    if (!xr.initialized) return;
    xr.rendering_hud = false;

    XrSwapchainImageReleaseInfo release_info = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    xr_check(xrReleaseSwapchainImage(xr.hud_swapchain.handle, &release_info), "xrReleaseSwapchainImage (HUD)");
}

bool vr_is_rendering_hud() { return xr.rendering_hud; }

#else // !ENABLE_DX11

// Stubs for non-D3D11 builds
bool vr_init() { return false; }
void vr_shutdown() {}
bool vr_begin_frame() { return false; }
void vr_end_frame() {}
void vr_begin_eye(int) {}
void vr_end_eye(int) {}
void vr_get_projection_matrix(int, float out[4][4]) { memset(out, 0, sizeof(float) * 16); }
void vr_get_view_matrix(int, float out[4][4]) { memset(out, 0, sizeof(float) * 16); }
bool vr_is_initialized() { return false; }
int vr_get_current_eye() { return 0; }
void vr_get_recommended_resolution(uint32_t* w, uint32_t* h) { *w = 0; *h = 0; }
uint32_t vr_get_refresh_rate() { return 90; }
float vr_get_world_scale() { return 1.0f; }
void vr_set_world_scale(float) {}
void vr_rebind_current_eye_target() {}
void vr_set_hud_commands(void*) {}
void* vr_get_hud_commands() { return nullptr; }
void vr_begin_hud() {}
void vr_end_hud() {}
bool vr_is_rendering_hud() { return false; }

#endif
