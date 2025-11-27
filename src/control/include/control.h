#pragma once
#include <pthread.h>
#include "crsf_protocol.h"
#include "config.h"
#include "common.h"

#define SEVERITY_NOTIFICATION  0
#define SEVERITY_WARN 1
#define SEVERITY_ERROR 2

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
    int bands;
    vrx_table_t vrx_table[VRX_TABLE_COUNT];
    pthread_mutex_t mutex;
    int status_severity;
    char *status_line;
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
