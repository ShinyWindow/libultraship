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

// Render target rebind (called when sub-framebuffer operations restore the main target)
void vr_rebind_current_eye_target();

// HUD overlay (rendered to a separate quad layer in front of the user)
void vr_set_hud_commands(void* commands);
void* vr_get_hud_commands();
void vr_begin_hud();
void vr_end_hud();
bool vr_is_rendering_hud();
