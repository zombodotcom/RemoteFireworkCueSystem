#pragma once
#include "status_model.h"
// Parse the controller's /api/status JSON into the box/seq fields of `out`.
// Returns false if json is null/empty or has no "boxes" array.
bool parseStatus(const char* json, StatusModel& out);

// Parse /api/events JSON ({"lastSeq":..,"events":[{seq,t,sev,msg},..]}) into out[],
// oldest->newest, up to maxOut. Returns the number written (0 if none/malformed).
int parseEvents(const char* json, LogEv* out, int maxOut);
