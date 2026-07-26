#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool VR_IsInitialized();
// Latch a pending VR<->flat mode toggle (CVar gVrEnabled). The game calls this once per game tick,
// BEFORE building the tick's display list, so a DL built for one mode is never drawn in the other.
void VR_ApplyModeRequest(void);
void VR_SetOverlayDisplayList(void* commands);

// Bracket the game's fixed-timestep logic update so the VR performance readout can separate it
// from render cost. Game logic runs once per 20 Hz tick on the same thread as the render passes,
// so its cost comes straight out of that tick's render budget; seeing it split out is the whole
// point. No-ops when VR is inactive.
void VR_GameTickBegin(void);
void VR_GameTickEnd(void);

// Flat-screen (2D) contexts — file select, pause menu. While enabled, the frame renders onto a
// world-locked floating panel placed in front of the player (instead of the stereo world), and the
// last world frame stays frozen-but-head-tracked behind it. The game sets this every frame.
void VR_SetFlatScreen(bool enabled);
bool VR_IsFlatScreen(void);

// First-person camera (game-side integration).
// The game pushes Link's head position each frame as the world-space anchor;
// the VR layer composes the view as anchor + HMD offset/orientation.
void VR_SetFirstPerson(bool enabled);
bool VR_GetFirstPerson(void);
void VR_SetCameraAnchor(float x, float y, float z);
// Link's standing eye height in game units, pushed each first-person frame. With auto world scale
// on, the VR layer derives units/meter from this and the player's real measured eye height, so the
// game ground matches the physical floor and child/adult swaps rescale automatically.
void VR_SetLinkEyeHeight(float units);
// Alyx-style comfort fade: 0 = clear, 1 = world layer black (menus/HUD unaffected). The game sets
// this each tick from how far the player's physical head sits beyond solid geometry.
void VR_SetViewFade(float fade);
// Third person: the game camera's facing (binang yaw) becomes the playspace's base orientation,
// so looking straight ahead in the headset looks where the stock camera looks (cutscenes too).
void VR_SetCameraYaw(int16_t yaw_binang);

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

// Roomscale 6DOF (physical walking moves Link's body, collision-swept). The game reads the desired
// per-frame body move (VR_GetRoomscaleDesired), collision-sweeps it, reports the achieved amount
// (VR_AddRoomscaleDisplacement), and pushes anchor = bodyHead - VR_GetRoomscaleOrigin. VR_ResetRoomscale
// re-zeros so the current physical position maps to Link's current body (recenter / scene / enable).
void VR_GetRoomscaleDesired(float out[2]);
void VR_AddRoomscaleDisplacement(float dx, float dz);
void VR_GetRoomscaleOrigin(float out[2]);
void VR_ResetRoomscale(void);
// Bound how far the camera may sit from Link's body horizontally (so the controller-driven camera
// stays within Link's collision; only physical 6DOF lean uses this slack). <= 0 disables.
void VR_ClampRoomscaleLean(float max_units);

// --- Motion controls ---
// Hand index.
#define VR_HAND_LEFT  0
#define VR_HAND_RIGHT 1
// Controller button bitmask (VR_GetControllerButton, per hand). Face buttons are per-hand:
// PRIMARY = A (right) / X (left); SECONDARY = B (right) / Y (left). Analog trigger/grip are also
// thresholded into the TRIGGER/GRIP bits so they read as digital buttons.
#define VR_BTN_TRIGGER    (1 << 0)
#define VR_BTN_GRIP       (1 << 1)
#define VR_BTN_PRIMARY    (1 << 2)
#define VR_BTN_SECONDARY  (1 << 3)
#define VR_BTN_THUMBCLICK (1 << 4)
#define VR_BTN_MENU       (1 << 5)

// Controller grip pose in game-world coords (eye/anchor frame): pos in game units, quat is x,y,z,w.
// Returns false (and identity) if that hand isn't tracked. Buttons/sticks/trigger/grip per hand.
bool     VR_GetHandPose(int hand, float pos[3], float quat[4]);
// Controller aim ray (runtime-calibrated pointing pose) in game-world coords: origin + unit
// forward direction. This is the ray for weapon aiming (slingshot/bow/hookshot).
bool     VR_GetAimRay(int hand, float pos[3], float dir[3]);
bool     VR_IsHandActive(int hand);
uint16_t VR_GetControllerButton(int hand);
void     VR_GetThumbstick(int hand, float* x, float* y);
float    VR_GetTrigger(int hand);
float    VR_GetGrip(int hand);
// Hand draw matrix (model-local -> game-world, engine MtxF layout) for pinning Link's hand limb to the
// controller. Includes Link's model scale (set via VR_SetHandScale). False if untracked.
bool     VR_GetHandMatrix(int hand, float out[4][4]);

// Live hand rendering. Set the model scale (Link's actor.scale) each frame; clear the hand-matrix
// registry each frame, then tag each hand limb's Mtx* so gfx_pc replaces it with the live controller
// pose per eye (full headset rate, no game-rate judder).
void     VR_SetHandScale(float s);
// Reflect that hand's geometry to flip handedness (when a controller drives Link's opposite-side hand
// model). Per hand: reflecting also mirrors held items' face designs (e.g. the shield crest), so the
// shield hand typically stays unmirrored. The game must also invert back-face culling for a mirrored
// hand. Axis via gVrHandMirrorAxis.
void     VR_SetHandMirror(int hand, bool mirror);
void     VR_RegisterHandMatrix(const void* mtx, int hand);
void     VR_ClearHandMatrices(void);

#ifdef __cplusplus
}
#endif
