#pragma once

#include <openvr.h>

struct VRSystem {
    vr::IVRSystem* system;
    vr::IVRCompositor* compositor;
    vr::IVRRenderModels* render_models;
    vr::TrackedDevicePose_t tracked_device_poses[vr::k_unMaxTrackedDeviceCount];
    vr::HmdMatrix34_t eye_positions[2];
    float eye_view_matrices[2][4][4];
    float eye_projection_matrices_converted[2][4][4];
    bool initialized;
    int current_eye;
};
