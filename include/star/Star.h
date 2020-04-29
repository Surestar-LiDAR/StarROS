/**
 * @author      : John
 * @date        : 2016-10-13 21:40
 */

#ifndef __SS_CONFIG_H
#define __SS_CONFIG_H

//#ifndef __enable_dynamic
//#define __dllexport
//#define __dllimport
//#define __dllhidden
//#else

#ifndef __dllexport
#if defined(_MSC_VER)
#define __dllexport __declspec(dllexport)
#define __dllimport __declspec(dllimport)
#define __dllhidden
#elif defined(__MINGW32__) || defined(__SYGWIN__)
#define __dllexport __attribute__((dllexport))
#define __dllimport __attribute__((dllimport))
#define __dllhidden __attribute__((visibility("hidden")))
#else
#define __dllexport
#define __dllimport
#define __dllhidden
#endif
#endif //__dllexport
//#endif

#ifndef __star_export
#if defined(star_EXPORTS)
#define __star_export __dllexport
#else
#define __star_export __dllimport
#endif
#define __star_hidden __dllhidden
#endif //__star_export
#ifdef _MSC_VER
#pragma warning(disable:4251)
#pragma warning(disable:4275)
#endif

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#endif

#if defined(__GNUC__)
#include <sys/types.h>
#endif

#if defined(_MSC_VER)
#if !defined (_SSIZE_T_DEFINED) && !defined (_SSIZE_T_DECLARED)
#define _SSIZE_T_DEFINED
#define _SSIZE_T_DECLARED
#ifdef _WIN64
typedef __int64 ssize_t;
#else
typedef long ssize_t;
#endif /* _WIN64 */
#endif /* _SSIZE_T_DEFINED */
#endif

#ifndef __CXX11__
#if (__cplusplus >= 201103) || (defined(_MSC_VER) && (_MSC_VER > 1700))
#define __CXX11__ 1
#else
#define __CXX11__ 0
#ifndef NULL
#define NULL 0
#endif
#define nullptr NULL
#define override
#define final
#define noexcept throw()
#endif
#endif

#if defined(_MSC_VER)
#define EXPORT_STATIC(name, ...)
#else
#define EXPORT_STATIC(names, ...) __attribute__((abi_tag(names)))
#endif

#ifndef __ARM_NEON__
#if defined(__ARM_NEON)
#define __ARM_NEON__ __ARM_NEON
#endif
#endif

#if defined(_DEBUG) || defined(DEBUG) || defined(__ENABLE_STAR_TRACE)
#define TRACE(fmt, ...) printf(fmt, __VA_ARGS__)
#endif

#ifndef TRACE
#define TRACE(fmt, ...)
#endif

#if defined(_WIN32) || defined (__USE_POSIX) || defined (__macos__)
#define __USE_STD_THREAD__
#endif

#define _FILE_OFFSET_BITS 64

#endif //__SS_CONFIG_H
