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
float vr_get_world_scale();
void vr_set_world_scale(float units_per_meter);

// Render target rebind (called when sub-framebuffer operations restore the main target)
void vr_rebind_current_eye_target();
