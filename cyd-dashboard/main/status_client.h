#pragma once
#include "status_model.h"   // (already) — LogEv lives here
void status_client_poll_once(StatusModel& model);
int  status_client_poll_events(uint32_t& sinceSeq, LogEv* out, int maxOut);
