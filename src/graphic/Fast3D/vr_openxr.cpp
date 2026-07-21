#define NOMINMAX

#include "vr_openxr.h"

#ifdef ENABLE_DX11

#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <unordered_map>

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

    // First-person camera
    bool first_person;        // When true, view is anchored to Link's head (game-driven)
    glm::vec3 anchor;         // Link's head this game frame, in game/world units (set by the game)
    glm::vec3 anchor_prev;    // Link's head the previous game frame (for sub-frame interpolation)
    bool anchor_initialized;  // False until the first anchor is pushed
    float interp_alpha;       // 0..1 blend between anchor_prev and anchor for the current render pass
    int16_t heading_offset;   // binang offset mapping HMD yaw -> game-world yaw (set at recenter)

    // Roomscale 6DOF: accumulated horizontal physical-walk displacement (game units, .x = world x,
    // .y = world z) that has been baked into Link's body position. The game advances it ONLY by the
    // body's collision-limited achieved move, and pushes anchor = bodyHead - roomscale_origin so the
    // eye stays continuous as the body slides under the head. See vr_roomscale_6dof plan.
    glm::vec2 roomscale_origin;

    // Motion controls (OpenXR action sets). hand index: 0 = left, 1 = right.
    XrActionSet action_set;
    XrPath hand_path[2];               // /user/hand/left, /user/hand/right
    XrAction grip_pose_action;         // POSE (per-hand subaction)
    XrAction aim_pose_action;          // POSE (per-hand subaction)
    XrAction trigger_action;           // FLOAT
    XrAction squeeze_action;           // FLOAT (grip)
    XrAction thumbstick_action;        // VECTOR2F
    XrAction thumbstick_click_action;  // BOOL
    XrAction primary_action;           // BOOL: A (right) / X (left)
    XrAction secondary_action;         // BOOL: B (right) / Y (left)
    XrAction menu_action;              // BOOL
    XrSpace grip_space[2];
    XrSpace aim_space[2];
    bool input_initialized;
    // Raw located view poses, preserved for compositor submission. The game-facing poses in `views`
    // get the artificial snap-turn applied; the compositor must instead see the physical head pose
    // the rendered image corresponds to (the turn is a world-space change, not a head-pose change).
    XrPosef submit_pose[2];
    // Per-frame controller state (raw, in OpenXR local space)
    bool hand_active[2];
    XrPosef grip_pose[2];
    XrPosef aim_pose[2];
    float trigger_value[2];
    float squeeze_value[2];
    float thumbstick_x[2];
    float thumbstick_y[2];
    uint16_t buttons[2];               // VR_BTN_* bitmask per hand

    // HUD overlay
    XrSpace view_space;
    struct EyeSwapchain hud_swapchain;
    uint32_t hud_image_index;
    void* hud_commands;
    bool rendering_hud;

    // Flat-screen mode: 2D contexts (file select, pause menu) render the whole frame onto a
    // world-locked floating panel instead of the stereo eyes. The last-rendered world frame keeps
    // being submitted behind it with its original pose, so it stays frozen-but-head-tracked.
    bool flat_screen;
    bool flat_screen_prev;
    XrPosef flat_pose; // panel pose in local_space (RAW tracking coords — quads bypass the snap-turn)
    struct EyeSwapchain screen_swapchain;
    uint32_t screen_image_index;
    bool rendering_screen;   // currently rendering into the screen swapchain (vs the HUD's)
    bool eyes_ever_rendered; // don't submit the projection layer before its swapchains have content

    // Desktop mirror: a copy of the left eye for display in the companion window. We can't sample the
    // swapchain image directly at present time (the runtime owns it once released), so the left eye is
    // copied here each frame while still acquired.
    ComPtr<ID3D11Texture2D> mirror_texture;
    ComPtr<ID3D11ShaderResourceView> mirror_srv;

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
// Motion controls: OpenXR action-set setup + per-frame sync
// --------------------------------------------------------------------------

// Create the gameplay action set, controller pose + input actions, suggest bindings for the common
// runtimes, attach to the session, and create per-hand pose spaces. Called once during vr_init after
// the reference space exists. Optional: on failure motion controls are disabled but the HMD works.
static bool setup_input() {
    XrActionSetCreateInfo set_ci = { XR_TYPE_ACTION_SET_CREATE_INFO };
    strcpy(set_ci.actionSetName, "gameplay");
    strcpy(set_ci.localizedActionSetName, "Gameplay");
    if (!xr_check(xrCreateActionSet(xr.instance, &set_ci, &xr.action_set), "xrCreateActionSet")) {
        return false;
    }

    xrStringToPath(xr.instance, "/user/hand/left", &xr.hand_path[0]);
    xrStringToPath(xr.instance, "/user/hand/right", &xr.hand_path[1]);

    auto make_action = [&](const char* name, const char* localized, XrActionType type, XrAction* out) -> bool {
        XrActionCreateInfo ci = { XR_TYPE_ACTION_CREATE_INFO };
        strcpy(ci.actionName, name);
        strcpy(ci.localizedActionName, localized);
        ci.actionType = type;
        ci.countSubactionPaths = 2;
        ci.subactionPaths = xr.hand_path;
        return xr_check(xrCreateAction(xr.action_set, &ci, out), "xrCreateAction");
    };

    bool ok = true;
    ok &= make_action("grip_pose", "Grip Pose", XR_ACTION_TYPE_POSE_INPUT, &xr.grip_pose_action);
    ok &= make_action("aim_pose", "Aim Pose", XR_ACTION_TYPE_POSE_INPUT, &xr.aim_pose_action);
    ok &= make_action("trigger", "Trigger", XR_ACTION_TYPE_FLOAT_INPUT, &xr.trigger_action);
    ok &= make_action("squeeze", "Squeeze", XR_ACTION_TYPE_FLOAT_INPUT, &xr.squeeze_action);
    ok &= make_action("thumbstick", "Thumbstick", XR_ACTION_TYPE_VECTOR2F_INPUT, &xr.thumbstick_action);
    ok &= make_action("thumbstick_click", "Thumbstick Click", XR_ACTION_TYPE_BOOLEAN_INPUT,
                      &xr.thumbstick_click_action);
    ok &= make_action("primary", "Primary Button", XR_ACTION_TYPE_BOOLEAN_INPUT, &xr.primary_action);
    ok &= make_action("secondary", "Secondary Button", XR_ACTION_TYPE_BOOLEAN_INPUT, &xr.secondary_action);
    ok &= make_action("menu", "Menu", XR_ACTION_TYPE_BOOLEAN_INPUT, &xr.menu_action);
    if (!ok) return false;

    auto path = [&](const char* s) -> XrPath {
        XrPath p = XR_NULL_PATH;
        xrStringToPath(xr.instance, s, &p);
        return p;
    };
    auto suggest = [&](const char* profile, std::vector<XrActionSuggestedBinding> binds) {
        XrInteractionProfileSuggestedBinding sb = { XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
        sb.interactionProfile = path(profile);
        sb.suggestedBindings = binds.data();
        sb.countSuggestedBindings = static_cast<uint32_t>(binds.size());
        xr_check(xrSuggestInteractionProfileBindings(xr.instance, &sb), "xrSuggestInteractionProfileBindings");
    };

    // Oculus Touch (Quest / Rift) — the most common.
    suggest("/interaction_profiles/oculus/touch_controller",
            { { xr.grip_pose_action, path("/user/hand/left/input/grip/pose") },
              { xr.grip_pose_action, path("/user/hand/right/input/grip/pose") },
              { xr.aim_pose_action, path("/user/hand/left/input/aim/pose") },
              { xr.aim_pose_action, path("/user/hand/right/input/aim/pose") },
              { xr.trigger_action, path("/user/hand/left/input/trigger/value") },
              { xr.trigger_action, path("/user/hand/right/input/trigger/value") },
              { xr.squeeze_action, path("/user/hand/left/input/squeeze/value") },
              { xr.squeeze_action, path("/user/hand/right/input/squeeze/value") },
              { xr.thumbstick_action, path("/user/hand/left/input/thumbstick") },
              { xr.thumbstick_action, path("/user/hand/right/input/thumbstick") },
              { xr.thumbstick_click_action, path("/user/hand/left/input/thumbstick/click") },
              { xr.thumbstick_click_action, path("/user/hand/right/input/thumbstick/click") },
              { xr.primary_action, path("/user/hand/left/input/x/click") },
              { xr.primary_action, path("/user/hand/right/input/a/click") },
              { xr.secondary_action, path("/user/hand/left/input/y/click") },
              { xr.secondary_action, path("/user/hand/right/input/b/click") },
              { xr.menu_action, path("/user/hand/left/input/menu/click") } });

    // Valve Index.
    suggest("/interaction_profiles/valve/index_controller",
            { { xr.grip_pose_action, path("/user/hand/left/input/grip/pose") },
              { xr.grip_pose_action, path("/user/hand/right/input/grip/pose") },
              { xr.aim_pose_action, path("/user/hand/left/input/aim/pose") },
              { xr.aim_pose_action, path("/user/hand/right/input/aim/pose") },
              { xr.trigger_action, path("/user/hand/left/input/trigger/value") },
              { xr.trigger_action, path("/user/hand/right/input/trigger/value") },
              { xr.squeeze_action, path("/user/hand/left/input/squeeze/value") },
              { xr.squeeze_action, path("/user/hand/right/input/squeeze/value") },
              { xr.thumbstick_action, path("/user/hand/left/input/thumbstick") },
              { xr.thumbstick_action, path("/user/hand/right/input/thumbstick") },
              { xr.thumbstick_click_action, path("/user/hand/left/input/thumbstick/click") },
              { xr.thumbstick_click_action, path("/user/hand/right/input/thumbstick/click") },
              { xr.primary_action, path("/user/hand/left/input/a/click") },
              { xr.primary_action, path("/user/hand/right/input/a/click") },
              { xr.secondary_action, path("/user/hand/left/input/b/click") },
              { xr.secondary_action, path("/user/hand/right/input/b/click") } });

    // KHR simple controller — universal fallback (pose + select + menu only).
    suggest("/interaction_profiles/khr/simple_controller",
            { { xr.grip_pose_action, path("/user/hand/left/input/grip/pose") },
              { xr.grip_pose_action, path("/user/hand/right/input/grip/pose") },
              { xr.aim_pose_action, path("/user/hand/left/input/aim/pose") },
              { xr.aim_pose_action, path("/user/hand/right/input/aim/pose") },
              { xr.primary_action, path("/user/hand/left/input/select/click") },
              { xr.primary_action, path("/user/hand/right/input/select/click") },
              { xr.menu_action, path("/user/hand/left/input/menu/click") },
              { xr.menu_action, path("/user/hand/right/input/menu/click") } });

    XrSessionActionSetsAttachInfo attach = { XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
    attach.countActionSets = 1;
    attach.actionSets = &xr.action_set;
    if (!xr_check(xrAttachSessionActionSets(xr.session, &attach), "xrAttachSessionActionSets")) {
        return false;
    }

    for (int h = 0; h < 2; h++) {
        XrActionSpaceCreateInfo as_ci = { XR_TYPE_ACTION_SPACE_CREATE_INFO };
        as_ci.poseInActionSpace = { { 0, 0, 0, 1 }, { 0, 0, 0 } };
        as_ci.subactionPath = xr.hand_path[h];
        as_ci.action = xr.grip_pose_action;
        xr_check(xrCreateActionSpace(xr.session, &as_ci, &xr.grip_space[h]), "xrCreateActionSpace (grip)");
        as_ci.action = xr.aim_pose_action;
        xr_check(xrCreateActionSpace(xr.session, &as_ci, &xr.aim_space[h]), "xrCreateActionSpace (aim)");
    }

    xr.input_initialized = true;
    spdlog::info("[VR] Motion-control input initialized");
    return true;
}

// Sync controller actions and locate the hand poses each frame. Called from vr_begin_frame after the
// views are located, with the same predicted display time. Safe to call before the session is focused
// (everything reads inactive -> zeros).
static void update_input() {
    if (!xr.input_initialized) return;

    XrActiveActionSet active = { xr.action_set, XR_NULL_PATH };
    XrActionsSyncInfo sync = { XR_TYPE_ACTIONS_SYNC_INFO };
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    if (!XR_SUCCEEDED(xrSyncActions(xr.session, &sync))) {
        return;
    }

    const XrTime t = xr.frame_state.predictedDisplayTime;

    for (int h = 0; h < 2; h++) {
        const XrPath hp = xr.hand_path[h];

        XrSpaceLocation loc = { XR_TYPE_SPACE_LOCATION };
        xrLocateSpace(xr.grip_space[h], xr.local_space, t, &loc);
        const bool valid = (loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
                           (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT);
        xr.hand_active[h] = valid;
        if (valid) {
            xr.grip_pose[h] = loc.pose;
        }

        XrSpaceLocation aloc = { XR_TYPE_SPACE_LOCATION };
        xrLocateSpace(xr.aim_space[h], xr.local_space, t, &aloc);
        if ((aloc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
            (aloc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
            xr.aim_pose[h] = aloc.pose;
        }

        auto get_float = [&](XrAction a) -> float {
            XrActionStateGetInfo gi = { XR_TYPE_ACTION_STATE_GET_INFO };
            gi.action = a;
            gi.subactionPath = hp;
            XrActionStateFloat st = { XR_TYPE_ACTION_STATE_FLOAT };
            if (XR_SUCCEEDED(xrGetActionStateFloat(xr.session, &gi, &st)) && st.isActive) {
                return st.currentState;
            }
            return 0.0f;
        };
        auto get_bool = [&](XrAction a) -> bool {
            XrActionStateGetInfo gi = { XR_TYPE_ACTION_STATE_GET_INFO };
            gi.action = a;
            gi.subactionPath = hp;
            XrActionStateBoolean st = { XR_TYPE_ACTION_STATE_BOOLEAN };
            if (XR_SUCCEEDED(xrGetActionStateBoolean(xr.session, &gi, &st)) && st.isActive) {
                return st.currentState == XR_TRUE;
            }
            return false;
        };

        xr.trigger_value[h] = get_float(xr.trigger_action);
        xr.squeeze_value[h] = get_float(xr.squeeze_action);

        XrActionStateGetInfo gi = { XR_TYPE_ACTION_STATE_GET_INFO };
        gi.action = xr.thumbstick_action;
        gi.subactionPath = hp;
        XrActionStateVector2f vst = { XR_TYPE_ACTION_STATE_VECTOR2F };
        if (XR_SUCCEEDED(xrGetActionStateVector2f(xr.session, &gi, &vst)) && vst.isActive) {
            xr.thumbstick_x[h] = vst.currentState.x;
            xr.thumbstick_y[h] = vst.currentState.y;
        } else {
            xr.thumbstick_x[h] = xr.thumbstick_y[h] = 0.0f;
        }

        // Bitmask. Analog trigger/grip are thresholded so they also read as digital buttons.
        uint16_t b = 0;
        if (xr.trigger_value[h] > 0.6f) b |= (1 << 0);            // VR_BTN_TRIGGER
        if (xr.squeeze_value[h] > 0.6f) b |= (1 << 1);            // VR_BTN_GRIP
        if (get_bool(xr.primary_action)) b |= (1 << 2);          // VR_BTN_PRIMARY
        if (get_bool(xr.secondary_action)) b |= (1 << 3);        // VR_BTN_SECONDARY
        if (get_bool(xr.thumbstick_click_action)) b |= (1 << 4); // VR_BTN_THUMBCLICK
        if (get_bool(xr.menu_action)) b |= (1 << 5);             // VR_BTN_MENU
        xr.buttons[h] = b;
    }
}

// --------------------------------------------------------------------------
// Artificial snap-turn: an accumulated world-space yaw (rotation + the translation that keeps the
// pivot fixed) applied to every game-facing pose. Never applied to compositor-submitted poses.
// --------------------------------------------------------------------------

static glm::quat g_turn_rot(1.0f, 0.0f, 0.0f, 0.0f);
static glm::vec3 g_turn_off(0.0f);

static XrPosef apply_turn(const XrPosef& p) {
    const glm::vec3 pos = g_turn_rot * glm::vec3(p.position.x, p.position.y, p.position.z) + g_turn_off;
    const glm::quat q =
        g_turn_rot * glm::quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    XrPosef out;
    out.position = { pos.x, pos.y, pos.z };
    out.orientation = { q.x, q.y, q.z, q.w };
    return out;
}

// Rotate the world by `degrees_right` (positive = player turns right) about the vertical axis
// through the player's current head position. Pivoting on the head keeps the player in place —
// any other pivot would translate them sideways as they turn. Called with this frame's raw views
// located but not yet turn-adjusted.
static void vr_apply_snap_turn(float degrees_right) {
    const float rad = degrees_right * (3.14159265358979323846f / 180.0f);
    // Right-handed yaw about +Y turns left, so turning right is the negative angle.
    const glm::quat r = glm::angleAxis(-rad, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::vec3 raw_center =
        0.5f * (glm::vec3(xr.views[0].pose.position.x, xr.views[0].pose.position.y, xr.views[0].pose.position.z) +
                glm::vec3(xr.views[1].pose.position.x, xr.views[1].pose.position.y, xr.views[1].pose.position.z));
    const glm::vec3 pivot = g_turn_rot * raw_center + g_turn_off; // where the head currently appears
    g_turn_rot = glm::normalize(r * g_turn_rot);
    g_turn_off = r * (g_turn_off - pivot) + pivot;
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
    xr.roomscale_origin = glm::vec2(0.0f);

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

    // Motion-control input (controller poses + buttons). Optional — the HMD works without it, so a
    // failure here just leaves input_initialized false and the VR_Get*Hand/Button APIs return empty.
    setup_input();

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

    // The game outputs gamma-encoded (sRGB) colors. The swapchain must be created with an
    // SRGB format so the compositor decodes them correctly; a UNORM swapchain makes the
    // compositor treat gamma values as linear and re-encode them, washing the image out.
    // Writes still go through a UNORM view (below) so the bits land in the texture verbatim.
    int64_t chosen_format = formats[0]; // fallback to first supported
    for (int64_t fmt : formats) {
        if (fmt == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) {
            chosen_format = fmt;
            break;
        }
    }
    if (chosen_format != DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) {
        // SRGB not available — pick UNORM as next best, accept the gamma mismatch for now
        for (int64_t fmt : formats) {
            if (fmt == DXGI_FORMAT_R8G8B8A8_UNORM) {
                chosen_format = fmt;
                break;
            }
        }
    }
    // OpenXR D3D11 swapchain textures are allocated typeless, so views may use either
    // variant of the format family. Using the UNORM variant for RTVs/SRVs stores and
    // reads the game's already-gamma-encoded output without any extra conversion.
    DXGI_FORMAT view_format = (chosen_format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB)
                                  ? DXGI_FORMAT_R8G8B8A8_UNORM
                                  : static_cast<DXGI_FORMAT>(chosen_format);
    spdlog::info("[VR] Swapchain format: {} (UNORM={}, SRGB={}), view format: {}",
                 chosen_format, (int)DXGI_FORMAT_R8G8B8A8_UNORM, (int)DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
                 (int)view_format);

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
            rtv_desc.Format = view_format;
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

    // --- Create desktop mirror texture (a copy of the left eye, shown in the companion window) ---
    {
        const auto& eye0 = xr.eye_swapchains[0];
        D3D11_TEXTURE2D_DESC desc = {};
        desc.Width = eye0.width;
        desc.Height = eye0.height;
        desc.MipLevels = 1;
        desc.ArraySize = 1;
        desc.Format = view_format;
        desc.SampleDesc.Count = 1;
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

        HRESULT hr = xr.d3d_device->CreateTexture2D(&desc, nullptr, xr.mirror_texture.ReleaseAndGetAddressOf());
        if (SUCCEEDED(hr)) {
            D3D11_SHADER_RESOURCE_VIEW_DESC srv_desc = {};
            srv_desc.Format = view_format;
            srv_desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
            srv_desc.Texture2D.MipLevels = 1;
            hr = xr.d3d_device->CreateShaderResourceView(xr.mirror_texture.Get(), &srv_desc,
                                                         xr.mirror_srv.ReleaseAndGetAddressOf());
        }
        if (FAILED(hr)) {
            // Non-fatal: the headset still renders, the companion window just won't show the mirror.
            spdlog::warn("[VR] Failed to create desktop mirror texture; companion window will be blank");
            xr.mirror_texture.Reset();
            xr.mirror_srv.Reset();
        } else {
            spdlog::info("[VR] Desktop mirror texture: {}x{}", eye0.width, eye0.height);
        }
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
            rtv_desc.Format = view_format;
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

    // --- Create flat-screen swapchain (whole-frame panel for 2D contexts: file select, pause) ---
    {
        auto& sc = xr.screen_swapchain;
        sc.width = 1280;
        sc.height = 960;
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

        if (!xr_check(xrCreateSwapchain(xr.session, &swapchain_ci, &sc.handle), "xrCreateSwapchain (screen)")) {
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
            rtv_desc.Format = view_format;
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
        spdlog::info("[VR] Screen swapchain: {}x{}, {} images", sc.width, sc.height, image_count);
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

    {
        auto& sc = xr.screen_swapchain;
        sc.rtvs.clear(); sc.dsvs.clear(); sc.depth_textures.clear(); sc.images.clear();
        if (sc.handle != XR_NULL_HANDLE) { xrDestroySwapchain(sc.handle); sc.handle = XR_NULL_HANDLE; }
    }
    xr.eyes_ever_rendered = false;
    xr.flat_screen = false;
    xr.flat_screen_prev = false;

    xr.mirror_srv.Reset();
    xr.mirror_texture.Reset();
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

    // Live-tunable world scale (game units per real-world meter). Higher = the world feels smaller;
    // together with the game-unit head offsets this fully controls perceived height above the ground.
    {
        float ws = CVarGetFloat("gVrWorldScale", 35.0f);
        if (ws < 5.0f) ws = 5.0f;
        if (ws > 200.0f) ws = 200.0f;
        xr.world_scale = ws;
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

    // Sync controllers + locate hand poses for this frame (motion controls).
    update_input();

    // Flat-screen panel placement: on entering a 2D context, drop the panel in front of the
    // player's current gaze. Uses the RAW located pose — quad layers are submitted in local_space
    // and never include the artificial snap-turn.
    if (xr.flat_screen && !xr.flat_screen_prev) {
        const XrPosef& vp = xr.views[0].pose;
        const glm::vec3 head(0.5f * (xr.views[0].pose.position.x + xr.views[1].pose.position.x),
                             0.5f * (xr.views[0].pose.position.y + xr.views[1].pose.position.y),
                             0.5f * (xr.views[0].pose.position.z + xr.views[1].pose.position.z));
        const glm::quat ho(vp.orientation.w, vp.orientation.x, vp.orientation.y, vp.orientation.z);
        glm::vec3 fwd = ho * glm::vec3(0.0f, 0.0f, -1.0f);
        fwd.y = 0.0f;
        const float len = glm::length(fwd);
        fwd = (len > 1e-4f) ? fwd / len : glm::vec3(0.0f, 0.0f, -1.0f);
        float dist = CVarGetFloat("gVrScreenDistance", 2.2f);
        if (dist < 0.5f) dist = 0.5f;
        const glm::vec3 pos = head + fwd * dist;
        // Yaw-only orientation, the quad's front (+Z) facing back at the player.
        const float qyaw = atan2f(-fwd.x, -fwd.z);
        const glm::quat q = glm::angleAxis(qyaw, glm::vec3(0.0f, 1.0f, 0.0f));
        xr.flat_pose.position = { pos.x, pos.y, pos.z };
        xr.flat_pose.orientation = { q.x, q.y, q.z, q.w };
    }
    xr.flat_screen_prev = xr.flat_screen;

    // Snap turn (right stick X): latch a discrete turn on a threshold crossing; the stick must
    // return to center before the next snap fires. Suspended in flat-screen mode, where the right
    // stick navigates menus (C-buttons) instead.
    if (xr.input_initialized && !xr.flat_screen && CVarGetInteger("gVrSnapTurnOn", 1)) {
        static int snap_latch = 0;
        const float sx = xr.thumbstick_x[1];
        if (snap_latch == 0 && fabsf(sx) > 0.6f) {
            snap_latch = (sx > 0.0f) ? 1 : -1;
            vr_apply_snap_turn(snap_latch * CVarGetFloat("gVrSnapTurnDegrees", 45.0f));
        } else if (snap_latch != 0 && fabsf(sx) < 0.3f) {
            snap_latch = 0;
        }
    }

    // Apply the accumulated snap-turn to every game-facing pose, preserving the raw view poses for
    // layer submission in vr_end_frame. Hand poses are only adjusted when freshly located this frame
    // (a stale pose already carries the previous turn and would be double-rotated). In flat-screen
    // mode the submit poses are NOT refreshed: the projection layer keeps re-submitting the last
    // world frame with the pose it was rendered from, so the frozen world stays world-locked.
    for (int eye = 0; eye < 2; eye++) {
        if (!xr.flat_screen) {
            xr.submit_pose[eye] = xr.views[eye].pose;
        }
        xr.views[eye].pose = apply_turn(xr.views[eye].pose);
    }
    for (int h = 0; h < 2; h++) {
        if (xr.hand_active[h]) {
            xr.grip_pose[h] = apply_turn(xr.grip_pose[h]);
            xr.aim_pose[h] = apply_turn(xr.aim_pose[h]);
        }
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
        // Submit the RAW physical pose, not the turn-adjusted one — reprojection must compare
        // against where the player's head actually is, or the compositor would fight the snap turn.
        projection_views[eye].pose = xr.submit_pose[eye];
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

    // Flat-screen quad (world-locked panel with the whole 2D frame: file select, pause menu)
    XrCompositionLayerQuad screen_layer = { XR_TYPE_COMPOSITION_LAYER_QUAD };
    screen_layer.space = xr.local_space;
    screen_layer.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    screen_layer.subImage.swapchain = xr.screen_swapchain.handle;
    screen_layer.subImage.imageRect.offset = { 0, 0 };
    screen_layer.subImage.imageRect.extent = {
        static_cast<int32_t>(xr.screen_swapchain.width),
        static_cast<int32_t>(xr.screen_swapchain.height)
    };
    screen_layer.subImage.imageArrayIndex = 0;
    screen_layer.pose = xr.flat_pose;
    {
        float sw = CVarGetFloat("gVrScreenSize", 2.4f);
        if (sw < 0.5f) sw = 0.5f;
        screen_layer.size = { sw, sw * 0.75f }; // 4:3, matching the swapchain
    }

    // Assemble layers back-to-front. The projection (world) layer is only submitted once its
    // swapchains have ever been rendered (at boot we go straight into flat-screen file select).
    const XrCompositionLayerBaseHeader* layers[3];
    uint32_t layer_count = 0;
    if (xr.eyes_ever_rendered) {
        layers[layer_count++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&projection_layer);
    }
    if (xr.flat_screen) {
        layers[layer_count++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&screen_layer);
    }
    layers[layer_count++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&hud_layer);

    XrFrameEndInfo end_info = { XR_TYPE_FRAME_END_INFO };
    end_info.displayTime = xr.frame_state.predictedDisplayTime;
    end_info.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;

    if (xr.frame_state.shouldRender) {
        end_info.layerCount = layer_count;
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
    xr.eyes_ever_rendered = true;

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

    // Grab the left eye for the desktop mirror while its swapchain image is still acquired — once
    // released below, the runtime owns the texture again and it's no longer safe to read.
    if (eye == 0) {
        vr_capture_mirror();
    }

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
    const float (*v)[4] = xr.view[eye];
    if (!xr.first_person) {
        memcpy(out, v, sizeof(float) * 16);
        return;
    }
    // First-person: anchored_view = T(-anchor) * view  (row-vector convention).
    // Pre-translating the world by -anchor places Link's head at the OpenXR local-space
    // origin, so the HMD's positional offset reads relative to Link's head and orientation
    // stays pure-HMD. Anchor is in game units, matching the world_scale-scaled HMD translation.
    // Rows 0-2 are unchanged; only row 3 (the translation row) folds in the anchor.
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 4; c++) {
            out[r][c] = v[r][c];
        }
    }
    // The game pushes the anchor at 20 fps; interpolate it to this render sub-frame with the same
    // alpha the engine uses for everything else, so the camera tracks the smoothly-rendered world.
    const glm::vec3 a = glm::mix(xr.anchor_prev, xr.anchor, xr.interp_alpha);
    const float ax = a.x, ay = a.y, az = a.z;
    for (int c = 0; c < 4; c++) {
        out[3][c] = v[3][c] - ax * v[0][c] - ay * v[1][c] - az * v[2][c];
    }
}

// Camera pose in game-world coords, matching the rendered (anchored) HMD view. The game feeds this
// into its own View (eye/lookAt/up) so frustum culling, audio panning and projected-position math
// align with what the player sees. Rendering is untouched (gfx_pc builds clip from the per-eye VR
// matrices and skips the game's lookAt in first-person).
//
// Derivation, no matrix inversion required: the rendered view maps a world point p to view space as
// R^-1 * (p - pos - anchor) (see vr_get_view_matrix), so the camera-to-world transform is
// translate(anchor + pos) * R. Hence eye = anchor + pos and the world forward/up are the HMD
// orientation's basis vectors (view space looks down -Z). Because the returned (eye, fwd, up) triple
// is self-consistent, feeding it back through the game's guLookAt reproduces the rendered view
// matrix exactly, sidestepping the OpenXR<->game axis-sign pitfalls that bit heading. Uses the
// center eye (average of the two eye poses). pos is scaled by world_scale to game units.
void vr_get_camera_pose(float eye[3], float fwd[3], float up[3]) {
    // Sensible identity defaults if a frame hasn't been located yet.
    eye[0] = eye[1] = eye[2] = 0.0f;
    fwd[0] = 0.0f; fwd[1] = 0.0f; fwd[2] = -1.0f;
    up[0] = 0.0f; up[1] = 1.0f; up[2] = 0.0f;
    if (!xr.initialized) return;

    // Center-eye position (midpoint of the two eyes), scaled to game units.
    glm::vec3 pos = 0.5f *
        (glm::vec3(xr.views[0].pose.position.x, xr.views[0].pose.position.y, xr.views[0].pose.position.z) +
         glm::vec3(xr.views[1].pose.position.x, xr.views[1].pose.position.y, xr.views[1].pose.position.z));
    pos *= xr.world_scale;

    // Center orientation: hemisphere-aligned, normalized average of the two eye quaternions (they're
    // near-identical, so an nlerp at 0.5 is plenty for culling). Bail to defaults if unset.
    const XrQuaternionf& q0r = xr.views[0].pose.orientation;
    const XrQuaternionf& q1r = xr.views[1].pose.orientation;
    glm::quat q0(q0r.w, q0r.x, q0r.y, q0r.z);
    glm::quat q1(q1r.w, q1r.x, q1r.y, q1r.z);
    if (glm::dot(q0, q1) < 0.0f) q1 = -q1;
    glm::quat q = q0 + q1;
    float qlen = glm::length(q);
    if (qlen < 1e-6f) return;
    q *= (1.0f / qlen);
    glm::mat3 R = glm::mat3_cast(q);

    // eye = Link's head anchor + the HMD's positional offset (the world point the render maps to the
    // view origin). Use the latest pushed anchor; for a per-game-frame culling query, sub-frame
    // interpolation isn't needed.
    glm::vec3 anchor = (xr.first_person && xr.anchor_initialized) ? xr.anchor : glm::vec3(0.0f);
    glm::vec3 e = anchor + pos;
    glm::vec3 f = R * glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 u = R * glm::vec3(0.0f, 1.0f, 0.0f);

    eye[0] = e.x; eye[1] = e.y; eye[2] = e.z;
    fwd[0] = f.x; fwd[1] = f.y; fwd[2] = f.z;
    up[0] = u.x; up[1] = u.y; up[2] = u.z;
}

// Vertical FOV (degrees) for the game's culling frustum, sized to cover the whole VR view. The
// game's native ~60-degree fovy is far narrower than the binocular VR field, so reusing it would
// cull geometry that's actually visible at the periphery (the very pop-in this fixes). Take the
// wider eye's vertical FOV and pad it generously; the culling projection derives its horizontal
// extent from the game's aspect ratio, so a wide fovy widens horizontal coverage too. Over-wide
// just draws slightly more geometry; too-narrow re-introduces edge pop-in, so we err wide.
float vr_get_culling_fovy() {
    const float kDefault = 100.0f;
    if (!xr.initialized) return kDefault;
    float vfov = 0.0f; // radians
    for (int eye = 0; eye < 2; eye++) {
        float v = xr.views[eye].fov.angleUp - xr.views[eye].fov.angleDown;
        if (v > vfov) vfov = v;
    }
    if (vfov <= 0.0f) return kDefault;
    const float kPad = 1.3f; // +30% headroom so nothing visible is culled
    float deg = glm::degrees(vfov) * kPad;
    if (deg < 90.0f) deg = 90.0f;
    if (deg > 160.0f) deg = 160.0f;
    return deg;
}

// --------------------------------------------------------------------------
// Roomscale 6DOF (physical walking moves Link's body, collision-swept)
// --------------------------------------------------------------------------

// Center-eye position (midpoint of the two eyes), scaled to game units. Zero if not located yet.
static glm::vec3 center_eye_pos_scaled() {
    if (!xr.initialized) return glm::vec3(0.0f);
    glm::vec3 pos = 0.5f *
        (glm::vec3(xr.views[0].pose.position.x, xr.views[0].pose.position.y, xr.views[0].pose.position.z) +
         glm::vec3(xr.views[1].pose.position.x, xr.views[1].pose.position.y, xr.views[1].pose.position.z));
    return pos * xr.world_scale;
}

// How far Link's body should try to move this frame to sit back under the head: the current head
// horizontal offset (game units) minus the displacement already baked into the body. The game
// rate-limits this, collision-sweeps it, and reports the achieved amount via the call below.
void vr_get_roomscale_desired(float out[2]) {
    glm::vec3 head = center_eye_pos_scaled();
    out[0] = head.x - xr.roomscale_origin.x;
    out[1] = head.z - xr.roomscale_origin.y;
}

// Advance the baked-in origin by the body's ACHIEVED horizontal move (collision-limited). Advancing
// by the achieved amount (not the desired amount) is what leaves blocked motion as a head-lean.
void vr_add_roomscale_displacement(float dx, float dz) {
    xr.roomscale_origin.x += dx;
    xr.roomscale_origin.y += dz;
}

// The baked-in origin (.x = world x, .y = world z), so the game can push anchor = bodyHead - origin.
void vr_get_roomscale_origin(float out[2]) {
    out[0] = xr.roomscale_origin.x;
    out[1] = xr.roomscale_origin.y;
}

// Re-zero roomscale so the player's current physical position maps to Link's current body position
// (desired -> 0, no body jerk). Called on recenter / first-person enable / scene change.
void vr_reset_roomscale() {
    glm::vec3 head = center_eye_pos_scaled();
    xr.roomscale_origin.x = head.x;
    xr.roomscale_origin.y = head.z;
}

// Clamp the head-lean — how far the camera sits horizontally from Link's body — to max_units, by
// advancing the baked-in origin toward the current head offset. The controller only ever moves the
// (collision-bounded) body, so this is what stops a large physical head offset (or wall-blocked
// motion) from floating the camera far past Link / out of bounds. max_units <= 0 disables it.
void vr_clamp_roomscale_lean(float max_units) {
    if (max_units <= 0.0f || !xr.initialized) return;
    glm::vec3 head = center_eye_pos_scaled();
    glm::vec2 residual(head.x - xr.roomscale_origin.x, head.z - xr.roomscale_origin.y);
    float len = glm::length(residual);
    if (len > max_units && len > 1e-4f) {
        glm::vec2 clamped = residual * (max_units / len);
        xr.roomscale_origin.x = head.x - clamped.x;
        xr.roomscale_origin.y = head.z - clamped.y;
    }
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

// --------------------------------------------------------------------------
// First-person camera
// --------------------------------------------------------------------------

void vr_set_first_person(bool enabled) {
    xr.first_person = enabled;
}

bool vr_is_first_person() {
    return xr.first_person;
}

void vr_set_camera_anchor(float x, float y, float z) {
    const glm::vec3 next(x, y, z);
    if (!xr.anchor_initialized) {
        xr.anchor = xr.anchor_prev = next;
        xr.anchor_initialized = true;
        return;
    }
    xr.anchor_prev = xr.anchor;
    xr.anchor = next;
    // Snap (skip interpolation) across large jumps like scene loads / warps, so the camera doesn't
    // smear across the cut. Normal movement is only a few units per game frame.
    const float kSnapDist = 200.0f;
    const glm::vec3 delta = next - xr.anchor_prev;
    if (glm::dot(delta, delta) > kSnapDist * kSnapDist) {
        xr.anchor_prev = next;
    }
}

void vr_set_interp_alpha(float alpha) {
    xr.interp_alpha = alpha;
}

// --------------------------------------------------------------------------
// Motion controls: accessors (hand: 0 = left, 1 = right)
// --------------------------------------------------------------------------

// Controller grip pose in game-world coords, composed the SAME way as the camera eye: world pos =
// anchor + grip_position * world_scale (interpolated anchor, so hands track the smoothly-rendered
// body), orientation = the controller orientation in the game-world frame (the same basis the camera
// uses). The game pushes the combined anchor (bodyHead - roomscale_origin) via vr_set_camera_anchor,
// so hands are automatically consistent with the eye + roomscale. out_quat is x,y,z,w. Returns false
// (and identity) if the hand isn't tracked.
bool vr_get_hand_pose(int hand, float out_pos[3], float out_quat[4]) {
    if (hand < 0 || hand > 1 || !xr.initialized || !xr.input_initialized || !xr.hand_active[hand]) {
        out_pos[0] = out_pos[1] = out_pos[2] = 0.0f;
        out_quat[0] = out_quat[1] = out_quat[2] = 0.0f;
        out_quat[3] = 1.0f;
        return false;
    }
    const XrPosef& p = xr.grip_pose[hand];
    const glm::vec3 anchor = (xr.first_person && xr.anchor_initialized)
                                 ? glm::mix(xr.anchor_prev, xr.anchor, xr.interp_alpha)
                                 : glm::vec3(0.0f);
    out_pos[0] = anchor.x + p.position.x * xr.world_scale;
    out_pos[1] = anchor.y + p.position.y * xr.world_scale;
    out_pos[2] = anchor.z + p.position.z * xr.world_scale;
    out_quat[0] = p.orientation.x;
    out_quat[1] = p.orientation.y;
    out_quat[2] = p.orientation.z;
    out_quat[3] = p.orientation.w;
    return true;
}

bool vr_is_hand_active(int hand) {
    return (hand >= 0 && hand <= 1) && xr.input_initialized && xr.hand_active[hand];
}

uint16_t vr_get_controller_buttons(int hand) {
    if (hand < 0 || hand > 1 || !xr.input_initialized) return 0;
    return xr.buttons[hand];
}

void vr_get_thumbstick(int hand, float* x, float* y) {
    if (hand < 0 || hand > 1 || !xr.input_initialized) {
        *x = *y = 0.0f;
        return;
    }
    *x = xr.thumbstick_x[hand];
    *y = xr.thumbstick_y[hand];
}

float vr_get_trigger(int hand) {
    if (hand < 0 || hand > 1 || !xr.input_initialized) return 0.0f;
    return xr.trigger_value[hand];
}

float vr_get_grip(int hand) {
    if (hand < 0 || hand > 1 || !xr.input_initialized) return 0.0f;
    return xr.squeeze_value[hand];
}

// Live hand-matrix registry: maps each frame's hand limb Mtx* to its controller index so gfx_pc can
// substitute a fresh controller pose per eye, bypassing the game-rate interpolation that makes the
// hands judder (the camera is smooth for the same reason — it's replaced live per eye). g_hand_scale
// is Link's model scale, folded into the hand matrix so the live-replaced hand renders at full size.
static std::unordered_map<const void*, int> g_hand_mtx_registry;
static float g_hand_scale = 1.0f;
static bool g_hand_mirror[2] = { false, false }; // per controller hand: reflect the hand geometry
                                                 // (flip handedness) when it drives Link's
                                                 // opposite-side hand model

// Hand draw matrix (model-local -> game-world) in the engine's row-vector MtxF layout, for pinning
// Link's hand limb to the controller. Same world position as vr_get_hand_pose (anchor + grip_pos *
// world_scale, so hands stay consistent with the camera + roomscale), orientation = controller
// orientation * a tunable calibration (gVrHandCal* CVars, degrees) so the held item lines up with the
// real controller. Does NOT include Link's model scale — the game applies actor.scale afterward.
// Layout matches pose_to_view_matrix (out[r][c] = glm column r, row c). false (+ identity) if untracked.
bool vr_get_hand_matrix(int hand, float out[4][4]) {
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            out[r][c] = (r == c) ? 1.0f : 0.0f;
    if (hand < 0 || hand > 1 || !xr.initialized || !xr.input_initialized || !xr.hand_active[hand]) {
        return false;
    }
    const XrPosef& p = xr.grip_pose[hand];
    const glm::vec3 anchor = (xr.first_person && xr.anchor_initialized)
                                 ? glm::mix(xr.anchor_prev, xr.anchor, xr.interp_alpha)
                                 : glm::vec3(0.0f);
    const glm::vec3 world_pos(anchor.x + p.position.x * xr.world_scale,
                              anchor.y + p.position.y * xr.world_scale,
                              anchor.z + p.position.z * xr.world_scale);
    glm::quat q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    const float kDeg = 3.14159265358979323846f / 180.0f;
    // Mirror axis: which model-local axis the reflection negates. The hand meshes' fingers/grip run
    // along model +X (the sword blade extends along hand-space +X, see the melee weapon tip/base in
    // z_player_lib.c), so the left<->right symmetry plane must KEEP X and flip the thumb axis —
    // default Z. Reflecting X itself (old default) turns the mesh inside-out instead of opposite-handed.
    int axis = CVarGetInteger("gVrHandMirrorAxis", 2);
    if (axis < 0 || axis > 2) axis = 2;
    const bool mirrored = g_hand_mirror[hand];
    // Calibration (model rest pose -> controller grip frame). Defaults were hand-tuned in-headset
    // against the MIRRORED sword hand on the right controller, which uses the mirror-conjugate of
    // these values (F * cal * F: the Euler component about the mirror axis is preserved, the other
    // two are negated). An UNMIRRORED hand also uses the conjugate regardless of controller: OpenXR
    // grip frames are defined per-hand (palm-relative), so mirror-symmetric physical poses report
    // the same orientation — an unreflected mesh attaches with the same rotation on either side.
    glm::vec3 calDeg(CVarGetFloat("gVrHandCalPitch", 88.0f), CVarGetFloat("gVrHandCalYaw", -100.0f),
                     CVarGetFloat("gVrHandCalRoll", 80.0f));
    // Positional offset (game units) in the controller grip frame, so the hand mesh can be nudged
    // to sit naturally on the controller; the conjugate reflects it (negate the mirror-axis component).
    glm::vec3 off(CVarGetFloat("gVrHandOffX", 0.0f), CVarGetFloat("gVrHandOffY", 0.0f),
                  CVarGetFloat("gVrHandOffZ", 0.0f));
    if (hand == 0 && CVarGetInteger("gVrHandLOverride", 1)) {
        // Fully independent left-controller tuning (values used literally, no conjugation).
        calDeg = glm::vec3(CVarGetFloat("gVrHandLCalPitch", -149.0f), CVarGetFloat("gVrHandLCalYaw", 76.0f),
                           CVarGetFloat("gVrHandLCalRoll", 30.0f));
        off = glm::vec3(CVarGetFloat("gVrHandLOffX", 0.0f), CVarGetFloat("gVrHandLOffY", 0.0f),
                        CVarGetFloat("gVrHandLOffZ", 0.0f));
    } else if (hand == 1 || !mirrored) {
        for (int k = 0; k < 3; k++) {
            if (k != axis) calDeg[k] = -calDeg[k];
        }
        off[axis] = -off[axis];
    }
    glm::quat cal = glm::quat(calDeg * kDeg);
    // Mirror = reflect the chosen model-local axis to flip the hand's handedness (the game also
    // inverts back-face culling for it).
    glm::vec3 sc(g_hand_scale);
    if (mirrored) {
        sc[axis] = -sc[axis];
    }
    glm::mat4 m = glm::translate(glm::mat4(1.0f), world_pos + q * off) * glm::mat4_cast(q * cal) *
                  glm::scale(glm::mat4(1.0f), sc);
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            out[r][c] = m[r][c];
    return true;
}

// Folds Link's model scale into the live hand matrix (the game sets this to actor.scale each frame).
void vr_set_hand_scale(float s) {
    g_hand_scale = s;
}

// Reflect that hand's geometry to flip its apparent handedness (set when the controller drives
// Link's opposite-side hand model). The game must also invert back-face culling for a mirrored hand.
void vr_set_hand_mirror(int hand, bool mirror) {
    if (hand >= 0 && hand <= 1) {
        g_hand_mirror[hand] = mirror;
    }
}

// The game tags each hand limb's per-frame Mtx* (register) and clears the registry each game frame;
// gfx_pc calls vr_lookup_hand_matrix per eye and, on a hit, uses the LIVE controller pose. addr is the
// limb's Mtx pointer, matching gfx_sp_matrix's addr argument.
void vr_register_hand_matrix(const void* mtx, int hand) {
    if (mtx) g_hand_mtx_registry[mtx] = hand;
}

void vr_clear_hand_matrices() {
    g_hand_mtx_registry.clear();
}

bool vr_lookup_hand_matrix(const void* mtx, float out[4][4]) {
    auto it = g_hand_mtx_registry.find(mtx);
    if (it == g_hand_mtx_registry.end()) return false;
    return vr_get_hand_matrix(it->second, out);
}

int16_t vr_get_head_yaw() {
    if (!xr.initialized) return 0;
    // Heading (yaw around the Y axis) of the headset, extracted from the HMD orientation.
    const XrQuaternionf& q = xr.views[0].pose.orientation;
    const float yaw = atan2f(2.0f * (q.w * q.y + q.x * q.z), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    // Convert radians -> binary angle (binang): pi maps to 0x8000.
    const float kPi = 3.14159265358979323846f;
    return static_cast<int16_t>(yaw / kPi * 32768.0f);
}

int16_t vr_get_heading_yaw() {
    if (!xr.initialized) return 0;
    // Steering must match what the player SEES. The first-person view is composed from the raw HMD
    // orientation — no recenter rotation is ever applied to the view — so the game-world look
    // direction is fully determined by the HMD pose alone. Derive the heading from the HMD forward
    // vector projected onto the horizontal plane: atan2(fx, fz) IS the game binang yaw (game yaw 0
    // faces +Z; movement applies sin->x, cos->z). This replaces the old Euler-angle extraction +
    // fudge constants, which skewed steering when the head pitched (up to ~15 deg looking down) and
    // added a recenter offset the view never used — the "walking sideways" bug: movement rotated
    // away from the look direction by (linkYaw - headYaw) captured at an arbitrary moment.
    static int16_t s_last_heading = 0;
    const XrQuaternionf& q = xr.views[0].pose.orientation;
    const glm::quat gq(q.w, q.x, q.y, q.z);
    const glm::vec3 fwd = gq * glm::vec3(0.0f, 0.0f, -1.0f);
    if (fwd.x * fwd.x + fwd.z * fwd.z > 1e-6f) {
        const float yaw = atan2f(fwd.x, fwd.z);
        s_last_heading = static_cast<int16_t>(yaw * (32768.0f / 3.14159265358979323846f));
    } // else: looking straight up/down, heading is degenerate — hold the last stable value
    const int16_t manual = static_cast<int16_t>(CVarGetInteger("gVrHeadingManualOffset", 0));
    return static_cast<int16_t>(s_last_heading + manual);
}

void vr_recenter_heading(int16_t link_yaw) {
    // Intentionally does NOT capture a steering offset anymore: the view never applies a recenter
    // rotation, so steering must not either — any captured offset rotates movement away from the
    // look direction (the old "walking sideways" bug). The game still calls this alongside
    // VR_ResetRoomscale when first-person (re)starts; there is simply nothing to do for heading.
    (void)link_yaw;
    xr.heading_offset = 0;
}

void vr_rebind_current_eye_target() {
    if (!xr.initialized || !xr.frame_began) return;

    // Restore whichever target is ACTUALLY being rendered: the flat-screen panel or the HUD when a
    // 2D pass is active (the pause menu runs framebuffer copies mid-pass — blindly rebinding an eye
    // here used to dump the inventory into the stale right-eye image), else the current eye.
    ID3D11RenderTargetView* rtv;
    ID3D11DepthStencilView* dsv;
    uint32_t height;
    if (xr.rendering_screen) {
        auto& sc = xr.screen_swapchain;
        rtv = sc.rtvs[xr.screen_image_index].Get();
        dsv = sc.dsvs[xr.screen_image_index].Get();
        height = sc.height;
    } else if (xr.rendering_hud) {
        auto& sc = xr.hud_swapchain;
        rtv = sc.rtvs[xr.hud_image_index].Get();
        dsv = sc.dsvs[xr.hud_image_index].Get();
        height = sc.height;
    } else {
        auto& sc = xr.eye_swapchains[xr.current_eye];
        uint32_t idx = xr.current_image_index[xr.current_eye];
        rtv = sc.rtvs[idx].Get();
        dsv = sc.dsvs[idx].Get();
        height = sc.height;
    }
    xr.d3d_context->OMSetRenderTargets(1, &rtv, dsv);
    gfx_d3d11_set_render_target_height(height);
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

// --------------------------------------------------------------------------
// Flat-screen mode (whole frame on a floating panel: file select, pause menu)
// --------------------------------------------------------------------------

void vr_set_flat_screen(bool enabled) {
    xr.flat_screen = enabled;
}

bool vr_get_flat_screen() {
    return xr.initialized && xr.flat_screen;
}

// Render the game's full frame into the screen swapchain. Reuses the HUD's "2D rendering" flag so
// gfx_pc uses the normal flat projection instead of the per-eye VR overrides.
void vr_begin_screen() {
    if (!xr.initialized) return;
    xr.rendering_hud = true; // gfx_pc's "2D target" flag: use the flat projection, not the VR eyes
    xr.rendering_screen = true;

    auto& sc = xr.screen_swapchain;
    XrSwapchainImageAcquireInfo acquire_info = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    uint32_t image_index = 0;
    xr_check(xrAcquireSwapchainImage(sc.handle, &acquire_info, &image_index), "xrAcquireSwapchainImage (screen)");
    xr.screen_image_index = image_index;

    XrSwapchainImageWaitInfo wait_info = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    wait_info.timeout = XR_INFINITE_DURATION;
    xr_check(xrWaitSwapchainImage(sc.handle, &wait_info), "xrWaitSwapchainImage (screen)");

    ID3D11RenderTargetView* rtv = sc.rtvs[image_index].Get();
    ID3D11DepthStencilView* dsv = sc.dsvs[image_index].Get();
    xr.d3d_context->OMSetRenderTargets(1, &rtv, dsv);

    float clear_color[] = { 0.0f, 0.0f, 0.0f, 1.0f }; // Opaque black
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

void vr_end_screen() {
    if (!xr.initialized) return;
    xr.rendering_hud = false;
    xr.rendering_screen = false;

    XrSwapchainImageReleaseInfo release_info = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    xr_check(xrReleaseSwapchainImage(xr.screen_swapchain.handle, &release_info), "xrReleaseSwapchainImage (screen)");
}

// Dimensions of whichever 2D target is currently being rendered (HUD quad or the flat-screen
// panel), so gfx_pc sizes the frame to the actual texture instead of assuming the HUD's.
void vr_get_2d_target_size(uint32_t* w, uint32_t* h) {
    if (xr.rendering_screen) {
        *w = xr.screen_swapchain.width;
        *h = xr.screen_swapchain.height;
    } else {
        *w = xr.hud_swapchain.width;
        *h = xr.hud_swapchain.height;
    }
}

// --------------------------------------------------------------------------
// Desktop mirror
// --------------------------------------------------------------------------

void vr_capture_mirror() {
    if (!xr.initialized || !xr.mirror_texture) return;

    auto& sc = xr.eye_swapchains[0];
    uint32_t idx = xr.current_image_index[0];
    if (idx >= sc.images.size()) return;

    ID3D11Texture2D* src = sc.images[idx].texture;
    if (src) {
        // Mirror texture was created with the same format/size as the eye swapchain image, so a
        // straight resource copy is valid (no shader blit needed).
        xr.d3d_context->CopyResource(xr.mirror_texture.Get(), src);
    }
}

void* vr_get_mirror_texture_id() {
    return xr.mirror_srv.Get();
}

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
void vr_set_first_person(bool) {}
bool vr_is_first_person() { return false; }
void vr_set_camera_anchor(float, float, float) {}
void vr_get_camera_pose(float eye[3], float fwd[3], float up[3]) {
    eye[0] = eye[1] = eye[2] = 0.0f;
    fwd[0] = 0.0f; fwd[1] = 0.0f; fwd[2] = -1.0f;
    up[0] = 0.0f; up[1] = 1.0f; up[2] = 0.0f;
}
float vr_get_culling_fovy() { return 100.0f; }
void vr_get_roomscale_desired(float out[2]) { out[0] = out[1] = 0.0f; }
void vr_add_roomscale_displacement(float, float) {}
void vr_get_roomscale_origin(float out[2]) { out[0] = out[1] = 0.0f; }
void vr_reset_roomscale() {}
void vr_clamp_roomscale_lean(float) {}
bool vr_get_hand_pose(int, float out_pos[3], float out_quat[4]) {
    out_pos[0] = out_pos[1] = out_pos[2] = 0.0f;
    out_quat[0] = out_quat[1] = out_quat[2] = 0.0f;
    out_quat[3] = 1.0f;
    return false;
}
bool vr_is_hand_active(int) { return false; }
uint16_t vr_get_controller_buttons(int) { return 0; }
void vr_get_thumbstick(int, float* x, float* y) { *x = *y = 0.0f; }
float vr_get_trigger(int) { return 0.0f; }
float vr_get_grip(int) { return 0.0f; }
bool vr_get_hand_matrix(int, float out[4][4]) {
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            out[r][c] = (r == c) ? 1.0f : 0.0f;
    return false;
}
void vr_set_hand_scale(float) {}
void vr_set_hand_mirror(int, bool) {}
void vr_register_hand_matrix(const void*, int) {}
void vr_clear_hand_matrices() {}
bool vr_lookup_hand_matrix(const void*, float out[4][4]) {
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            out[r][c] = (r == c) ? 1.0f : 0.0f;
    return false;
}
int16_t vr_get_head_yaw() { return 0; }
int16_t vr_get_heading_yaw() { return 0; }
void vr_recenter_heading(int16_t) {}
void vr_set_interp_alpha(float) {}
void vr_rebind_current_eye_target() {}
void vr_set_hud_commands(void*) {}
void* vr_get_hud_commands() { return nullptr; }
void vr_begin_hud() {}
void vr_end_hud() {}
bool vr_is_rendering_hud() { return false; }
void vr_set_flat_screen(bool) {}
bool vr_get_flat_screen() { return false; }
void vr_begin_screen() {}
void vr_end_screen() {}
void vr_get_2d_target_size(uint32_t* w, uint32_t* h) { *w = 1024; *h = 768; }
void vr_capture_mirror() {}
void* vr_get_mirror_texture_id() { return nullptr; }

#endif
