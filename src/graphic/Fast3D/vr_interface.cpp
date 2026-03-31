#include "vr_interface.h"
#include "gfx_pc.h"
#include "vr_system_internal.h"

extern "C" {

bool VR_IsInitialized() {
    return GetVRSystem()->initialized;
}

// bool VR_GetEyeToHeadMatrix(int eye, float outMatrix[3][4]) {
//     const VRSystem* vr = GetVRSystem();
//     if (!vr->initialized) return false;

//     const vr::HmdMatrix34_t& mat = vr->eye_positions[eye];
//     for (int r = 0; r < 3; r++)
//         for (int c = 0; c < 4; c++)
//             outMatrix[r][c] = mat.m[r][c];
    
//     return true;
// }

}
