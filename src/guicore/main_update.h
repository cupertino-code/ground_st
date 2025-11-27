#pragma once

#if defined(cplusplus) || defined(__cplusplus)
extern "C" {
#endif
void update_status(void);
void set_status_line(int severity, const char *status);
#if defined(cplusplus) || defined(__cplusplus)
}
#endif
