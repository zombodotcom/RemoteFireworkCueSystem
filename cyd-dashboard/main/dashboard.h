#pragma once
#include "status_model.h"
void dashboard_create(void);
void dashboard_update(const StatusModel& m);
void dashboard_set_events(const LogEv* evs, int n);
