#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool VR_IsInitialized();
void VR_SetOverlayDisplayList(void* commands);

#ifdef __cplusplus
}
#endif
