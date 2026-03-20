#ifndef LF_HARNESS_CORE_H
#define LF_HARNESS_CORE_H

#include <stdint.h>

#include "lf_harness_common.h"

void LFH_Core_Reset(const LFH_Scenario *scenario, const LFH_TestConfig *cfg);
void LFH_Core_ReadSensorNormAndRaw(double out_norm[LF_SENSOR_COUNT], uint16_t out_raw[LF_SENSOR_COUNT]);
LFH_LineState LFH_Core_EstimateLineState(const double sensor_norm[LF_SENSOR_COUNT], double threshold);

void LFH_Core_AdvanceTime(double dt_sec);
double LFH_Core_UpdatePoseFromCommand(double dt_sec);
int16_t LFH_Core_GetLeftCommand(void);
int16_t LFH_Core_GetRightCommand(void);

#endif
