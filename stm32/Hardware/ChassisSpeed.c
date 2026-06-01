#include "ChassisSpeed.h"
#include "config.h"

#ifndef ENCODER_QUADRATURE
#define ENCODER_QUADRATURE 4.0f
#endif

#ifndef PI
#define PI 3.14159265359f
#endif

int32_t ChassisSpeed_DeltaToMmps(int16_t delta_ticks, uint16_t dt_ms)
{
    float ticks_per_rev;
    float wheel_circ_mm;
    float dist_mm;
    float speed_mmps;

    if (dt_ms == 0)
    {
        return 0;
    }

    ticks_per_rev = (float)ENCODER_PPR * (float)GEAR_RATIO * ENCODER_QUADRATURE;
    wheel_circ_mm = PI * WHEEL_DIAMETER * 1000.0f;

    dist_mm = ((float)delta_ticks / ticks_per_rev) * wheel_circ_mm;
    speed_mmps = dist_mm * 1000.0f / (float)dt_ms;

    if (speed_mmps >= 0.0f)
    {
        return (int32_t)(speed_mmps + 0.5f);
    }
    else
    {
        return (int32_t)(speed_mmps - 0.5f);
    }
}
