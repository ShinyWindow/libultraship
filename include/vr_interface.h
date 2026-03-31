#pragma once

#ifdef __cplusplus
extern "C" {
#endif

bool VR_IsInitialized();

// // Pose getters
// bool VR_GetHMDPose(float outMatrix[4][4]);
// bool VR_GetControllerPose(int index, float outMatrix[4][4]);

// // Controller indices
// int VR_GetLeftControllerIndex();
// int VR_GetRightControllerIndex();

// // Input state
// bool VR_IsButtonPressed(int index, int button);      // EVRButtonId enum value
// bool VR_IsButtonTouched(int index, int button);
// float VR_GetAxisValue(int index, int axis);          // 0: trackpad x, 1: y, etc.

#ifdef __cplusplus
}
#endif
