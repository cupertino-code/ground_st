#pragma once

#if defined(cplusplus) || defined(__cplusplus)
extern "C" {
#endif
struct global_status;
typedef struct global_status global_status_t;

int crsf_start(global_status_t *status_ptr, app_config_t *app_config);
void crsf_stop(void);
void crsf_send_vrxtable(void);
#if defined(cplusplus) || defined(__cplusplus)
}
#endif
