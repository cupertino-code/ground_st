#pragma once
#include "crsf_protocol.h"
#include "config.h"
#include "common.h"

typedef struct global_status {
    int updated;
    int angle;
    int power_status;
    int connect_status;
    int vbat;
    int encoder_cnt;
    int switch_status;
    int recording;
    int channels_updated;
    crsf_channels_t channels;
    uint64_t rc_packets_good;
    uint64_t rc_packets;
    uint64_t rc_bytes;
    uint64_t rc_errs;
} global_status_t;

#if defined(cplusplus) || defined(__cplusplus)
extern "C" {
#endif

int control_start(global_status_t *status_ptr, app_config_t *app_config);
void control_stop(void);
void control_send_message(void);

#if defined(cplusplus) || defined(__cplusplus)
}
#endif
