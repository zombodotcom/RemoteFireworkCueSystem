#pragma once
#include "status_model.h"
#ifdef __cplusplus
extern "C" {
#endif
void espnow_rx_start(void);
void espnow_rx_snapshot(StatusModel* out, unsigned long nowMs);
int  espnow_rx_events(LogEv* out, int maxOut);
#ifdef __cplusplus
}
#endif
