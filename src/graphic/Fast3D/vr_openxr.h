#pragma once

#include <stdbool.h>
#include <stdint.h>

// Lifecycle
bool vr_init();
void vr_shutdown();

// Per-frame
bool vr_begin_frame();
void vr_end_frame();

// Per-eye
void vr_begin_eye(int eye);
void vr_end_eye(int eye);

// Matrix queries (used by gfx_pc.cpp matrix injection)
void vr_get_projection_matrix(int eye, float out[4][4]);
void vr_get_view_matrix(int eye, float out[4][4]);

// State queries
bool vr_is_initialized();
int vr_get_current_eye();
void vr_get_recommended_resolution(uint32_t* width, uint32_t* height);
// Headset display refresh rate in Hz (e.g. 72/90/120). Used to pace the game's fixed-timestep
// logic via the interpolation system. Returns a sane default before the first frame is located.
uint32_t vr_get_refresh_rate();
float vr_get_world_scale();
void vr_set_world_scale(float units_per_meter);

// First-person camera
void vr_set_first_person(bool enabled);
bool vr_is_first_person();
void vr_set_camera_anchor(float x, float y, float z);
int16_t vr_get_head_yaw();

// Camera unification (Phase 3): report the rendered HMD pose to the game so its CPU-side camera
// systems (frustum culling, audio panning, projected-position/LOD) agree with what the player sees.
// vr_get_camera_pose returns the center-eye pose in game-world coords (eye position + forward/up
// unit direction vectors). vr_get_culling_fovy returns a vertical FOV (degrees) wide enough to cover
// the whole binocular VR view, so peripheral geometry isn't culled. Only meaningful while
// first-person is active.
void vr_get_camera_pose(float eye[3], float fwd[3], float up[3]);
float vr_get_culling_fovy();

// Roomscale 6DOF (physical translation moves Link's body, collision-swept). roomscale_origin is the
// horizontal physical-walk displacement (game units) baked into the body; the game advances it only
// by the body's collision-limited achieved move. See vr_roomscale_6dof plan.
void vr_get_roomscale_desired(float out[2]);
void vr_add_roomscale_displacement(float dx, float dz);
void vr_get_roomscale_origin(float out[2]);
void vr_reset_roomscale();
// Bound how far the camera may sit from Link's body horizontally (comfort + keeps the controller-
// driven camera within Link's collision; only physical lean uses this slack). <= 0 disables.
void vr_clamp_roomscale_lean(float max_units);

// HMD-driven heading (Phase 2)
int16_t vr_get_heading_yaw();
void vr_recenter_heading(int16_t link_yaw);

// Sub-frame interpolation factor (0..1) for the current render pass, so the camera anchor can be
// interpolated between game frames in lockstep with the rest of the interpolated world.
void vr_set_interp_alpha(float alpha);

// Render target rebind (called when sub-framebuffer operations restore the main target)
void vr_rebind_current_eye_target();

// HUD overlay (rendered to a separate quad layer in front of the user)
void vr_set_hud_commands(void* commands);
void* vr_get_hud_commands();
void vr_begin_hud();
void vr_end_hud();
bool vr_is_rendering_hud();

// Desktop mirror: copy the rendered left eye into a sampleable texture so the companion window can
// display what the headset sees (and ImGui can composite the menu on top of it). vr_capture_mirror
// must run while the left-eye swapchain image is still acquired (i.e. before vr_end_eye(0) releases
// it). vr_get_mirror_texture_id returns the SRV as an ImTextureID-compatible pointer, or null if the
// mirror isn't available.
void vr_capture_mirror();
void* vr_get_mirror_texture_id();
