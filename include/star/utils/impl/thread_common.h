/**
 * Copyright (c) 2019 QuantumCTek. All rights reserved.
 * @author      : John 
 * @date        : 2019-07-31
 * @version     : 1.0.0
 */

#ifndef __STAR_SDK_UTILS_IMPL_THREAD_COMMON_H
#define __STAR_SDK_UTILS_IMPL_THREAD_COMMON_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* __thread_t;
typedef void (*__thread_entry)(void*);

__thread_t __thread_self();
__thread_t __thread_create(__thread_entry __entry, void *data, int stack_size);
bool __thread_free(__thread_t __handle);
bool __thread_start(__thread_t __handle);
bool __thread_abort(__thread_t __handle);
bool __thread_detach(__thread_t __handle);
bool __thread_join(__thread_t __handle);

void __thread_yield();
void __thread_msleep(int64_t time);
void __thread_usleep(int64_t time);

typedef void* __thread_mutex_t;
__thread_mutex_t __thread_mutex_create();
bool __thread_mutex_free(__thread_mutex_t __handle);
bool __thread_mutex_lock(__thread_mutex_t __handle);
bool __thread_mutex_try_lock(__thread_mutex_t __handle);
bool __thread_mutex_try_lock_for(__thread_mutex_t __handle, int64_t milliseconds);
bool __thread_mutex_unlock(__thread_mutex_t __handle);

typedef void* __thread_cond_t;
enum __cv_status {
    __cv_error,
    __cv_timeout,
    __cv_no_timeout,
};
__thread_cond_t __thread_cond_create();
bool __thread_cond_free(__thread_cond_t __handle);
bool __thread_cond_broadcast(__thread_cond_t __handle);
bool __thread_cond_signal(__thread_cond_t __handle);
bool __thread_cond_wait(__thread_cond_t __handle, __thread_mutex_t __mutex);
__cv_status __thread_cond_timed_wait(__thread_cond_t __handle, __thread_mutex_t __mutex, int64_t milliseconds);

#ifdef __cplusplus
}
#endif

#endif //__STAR_SDK_UTILS_IMPL_THREAD_COMMON_H
