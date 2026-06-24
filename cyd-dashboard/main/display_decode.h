#pragma once
#include "status_model.h"
#include "protocol.h"
void applyDisplayStatus(const fw::DisplayStatusPacket& p, StatusModel& m);
void toLogEv(const fw::DisplayEventPacket& p, LogEv& e);
