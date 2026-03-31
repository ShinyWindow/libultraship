#include "vr_interface.h"
#include "vr_openxr.h"

extern "C" {

bool VR_IsInitialized() {
    return vr_is_initialized();
}

void VR_SetOverlayDisplayList(void* commands) {
    vr_set_hud_commands(commands);
}

}
