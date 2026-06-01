#include "FlytProtocol.h"
#include <string.h>

uint8_t FlytProtocol_Xor(const uint8_t *buf, uint16_t len)
{
    uint8_t cs = 0;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        cs ^= buf[i];
    }

    return cs;
}

uint8_t FlytProtocol_ParseDownlink(const uint8_t *frame, FlytCmd_t *cmd)
{
    uint8_t cs;

    if (frame == 0 || cmd == 0)
    {
        return 0;
    }

    if (frame[0] != FLYT_DOWN_STX1 || frame[1] != FLYT_DOWN_STX2)
    {
        return 0;
    }

    cs = FlytProtocol_Xor(frame, FLYT_DOWNLINK_LEN - 1);
    if (cs != frame[FLYT_DOWNLINK_LEN - 1])
    {
        return 0;
    }

    memcpy(&cmd->linear_mps, &frame[2], 4);
    memcpy(&cmd->angular_radps, &frame[6], 4);
    cmd->valid = 1;

    return 1;
}

void FlytProtocol_PackUplink(int32_t left_total_ticks,
                             int32_t right_total_ticks,
                             float gyro_z_dps,
                             uint8_t status,
                             uint8_t out_frame[FLYT_UPLINK_LEN])
{
    out_frame[0] = FLYT_UP_STX1;
    out_frame[1] = FLYT_UP_STX2;

    memcpy(&out_frame[2], &left_total_ticks, 4);
    memcpy(&out_frame[6], &right_total_ticks, 4);
    memcpy(&out_frame[10], &gyro_z_dps, 4);

    out_frame[14] = status;
    out_frame[15] = FlytProtocol_Xor(out_frame, FLYT_UPLINK_LEN - 1);
}
