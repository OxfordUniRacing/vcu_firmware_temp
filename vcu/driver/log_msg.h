#ifndef LOG_MSG_H
#define LOG_MSG_H

#include "FreeRTOS.h"
#include "stream_buffer.h"

enum { LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG, LOG_JUST };

#define log_debug(...) log_log(LOG_DEBUG, __VA_ARGS__)
#define log_info(...)  log_log(LOG_INFO,  __VA_ARGS__)
#define log_warn(...)  log_log(LOG_WARN,  __VA_ARGS__)
#define log_error(...) log_log(LOG_ERROR, __VA_ARGS__)
//added
#define log_just(...) log_log_just(LOG_JUST, __VA_ARGS__)

extern StreamBufferHandle_t msglog_buffer;
void log_init(void);
void log_log(int level, const char *fmt, ...);
void log_log_just(int level, const char *fmt, ...);

#endif // LOG_MSG_H
