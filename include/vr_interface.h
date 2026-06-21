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

#ifdef __cplusplus
}
#endif
