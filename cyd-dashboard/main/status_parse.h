#pragma once
#include "status_model.h"
// Parse the controller's /api/status JSON into the box/seq fields of `out`.
// Returns false if json is null/empty or has no "boxes" array.
bool parseStatus(const char* json, StatusModel& out);
