#ifndef _LOG_H_
#define _LOG_H_

#include <stdarg.h>
#include <stdio.h>
#include <sys/time.h>
#ifndef __USE_XOPEN
#define __USE_XOPEN
#endif
#include <time.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <string>

int64_t getCurrentTime_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

#define LOG_BUF_SIZE 1024
void COUT_(std::string log_level, const char* format_str, ...) {
    va_list ap;
    char buf[LOG_BUF_SIZE];
    va_start(ap, format_str);
    size_t size = vsnprintf(buf, LOG_BUF_SIZE, format_str, ap);
    std::string dst(buf, buf + size);
    dst = log_level + ": " + dst;
    std::cout << dst << std::endl;
    va_end(ap);
    return;
}
#ifndef LOGI
#define LOGI(...) COUT_("INFO", __VA_ARGS__);
#endif
#ifndef LOGD
#define LOGD(...) COUT_("DEBUG", __VA_ARGS__);
#endif
#ifndef LOGW
#define LOGW(...) COUT_("WARNING", __VA_ARGS__);
#endif
#ifndef LOGE
#define LOGE(...) COUT_("ERROR", __VA_ARGS__);
#endif

#ifndef LOGD_IF
#define LOGD_IF(cond, ...) ((cond) ? ((void)LOGD(__VA_ARGS__)) : (void)0)
#endif

#endif  // _LOG_H_

