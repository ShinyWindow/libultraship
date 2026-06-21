#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool VR_IsInitialized();
void VR_SetOverlayDisplayList(void* commands);

// First-person camera (game-side integration).
// The game pushes Link's head position each frame as the world-space anchor;
// the VR layer composes the view as anchor + HMD offset/orientation.
void VR_SetFirstPerson(bool enabled);
bool VR_GetFirstPerson(void);
void VR_SetCameraAnchor(float x, float y, float z);

// HMD yaw as a binary angle (binang), for driving gameplay heading. (Phase 2)
int16_t VR_GetHeadYaw(void);

// HMD-driven heading. VR_GetHeadingYaw returns the head yaw mapped into game-world space
// (recenter offset + invert/manual-offset CVars applied) for use as Link's steering yaw.
// VR_RecenterHeading captures the offset so that the player's current physical facing maps
// to linkYaw (the player's current in-game facing).
int16_t VR_GetHeadingYaw(void);
void    VR_RecenterHeading(int16_t linkYaw);

// Camera unification (Phase 3). VR_GetCameraPose returns the rendered HMD pose in game-world coords
// (eye position + forward/up unit vectors); VR_GetCullingFovy returns a vertical FOV (degrees) wide
// enough to cover the binocular VR view. The game feeds these into its View so CPU-side systems
// (frustum culling, audio panning, projected-position/LOD) match what the player sees. Rendering is
// unaffected. Only meaningful while first-person is active.
void  VR_GetCameraPose(float eye[3], float fwd[3], float up[3]);
float VR_GetCullingFovy(void);

#ifdef __cplusplus
}
#endif
