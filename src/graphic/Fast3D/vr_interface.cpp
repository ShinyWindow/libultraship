#include "vr_interface.h"
#include "vr_openxr.h"

extern "C" {

bool VR_IsInitialized() {
    return vr_is_initialized();
}

}
