#include "vr_interface.h"
#include "vr_openxr.h"

extern "C" {

bool VR_IsInitialized() {
    return vr_is_initialized();
}

void VR_SetOverlayDisplayList(void* commands) {
    vr_set_hud_commands(commands);
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

}
