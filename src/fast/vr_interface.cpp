#include "vr_interface.h"
#include "fast/vr_openxr.h"

#include <chrono>

namespace {
std::chrono::steady_clock::time_point g_game_tick_start;
} // namespace

extern "C" {

void VR_GameTickBegin(void) {
    if (vr_is_initialized()) {
        g_game_tick_start = std::chrono::steady_clock::now();
    }
}

void VR_GameTickEnd(void) {
    if (vr_is_initialized()) {
        vr_report_game_tick_ms(
            std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - g_game_tick_start).count());
    }
}

void VR_SetFlatScreen(bool enabled) {
    vr_set_flat_screen(enabled);
}

bool VR_IsFlatScreen(void) {
    return vr_get_flat_screen();
}

bool VR_IsInitialized() {
    return vr_is_initialized();
}

void VR_ApplyModeRequest(void) {
    vr_apply_mode_request();
}

void VR_SetOverlayDisplayList(void* commands) {
    vr_set_hud_commands(commands);
}

void VR_SetCameraYaw(int16_t yaw_binang) {
    vr_set_camera_yaw(yaw_binang);
}

void VR_SetFirstPerson(bool enabled) {
    vr_set_first_person(enabled);
}

bool VR_GetFirstPerson(void) {
    return vr_is_first_person();
}

void VR_SetCameraAnchor(float x, float y, float z) {
    vr_set_camera_anchor(x, y, z);
}

int16_t VR_GetHeadYaw(void) {
    return vr_get_head_yaw();
}

int16_t VR_GetHeadingYaw(void) {
    return vr_get_heading_yaw();
}

void VR_RecenterHeading(int16_t linkYaw) {
    vr_recenter_heading(linkYaw);
}

void VR_GetCameraPose(float eye[3], float fwd[3], float up[3]) {
    vr_get_camera_pose(eye, fwd, up);
}

float VR_GetCullingFovy(void) {
    return vr_get_culling_fovy();
}

void VR_GetRoomscaleDesired(float out[2]) {
    vr_get_roomscale_desired(out);
}

void VR_AddRoomscaleDisplacement(float dx, float dz) {
    vr_add_roomscale_displacement(dx, dz);
}

void VR_GetRoomscaleOrigin(float out[2]) {
    vr_get_roomscale_origin(out);
}

void VR_ResetRoomscale(void) {
    vr_reset_roomscale();
}

void VR_ClampRoomscaleLean(float max_units) {
    vr_clamp_roomscale_lean(max_units);
}

bool VR_GetHandPose(int hand, float pos[3], float quat[4]) {
    return vr_get_hand_pose(hand, pos, quat);
}

bool VR_IsHandActive(int hand) {
    return vr_is_hand_active(hand);
}

uint16_t VR_GetControllerButton(int hand) {
    return vr_get_controller_buttons(hand);
}

void VR_GetThumbstick(int hand, float* x, float* y) {
    vr_get_thumbstick(hand, x, y);
}

float VR_GetTrigger(int hand) {
    return vr_get_trigger(hand);
}

float VR_GetGrip(int hand) {
    return vr_get_grip(hand);
}

bool VR_GetHandMatrix(int hand, float out[4][4]) {
    return vr_get_hand_matrix(hand, out);
}

void VR_SetHandScale(float s) {
    vr_set_hand_scale(s);
}

void VR_SetHandMirror(int hand, bool mirror) {
    vr_set_hand_mirror(hand, mirror);
}

void VR_RegisterHandMatrix(const void* mtx, int hand) {
    vr_register_hand_matrix(mtx, hand);
}

void VR_ClearHandMatrices(void) {
    vr_clear_hand_matrices();
}

}
