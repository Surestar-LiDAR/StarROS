/**
 * Copyright (c), 2017-2018 QuantumCTek. All rights reserved.
 * @file        : bytes_order.h
 * @project     : QCTekMidware
 * @author      : John 
 * @date        : 2018-06-01
 * @version     : 1.0.0
 * @brief       :
 */
#ifndef __STAR_SDK_UTILS_BYTES_ORDER_H
#define __STAR_SDK_UTILS_BYTES_ORDER_H

#include <cstdint>
#include <cstddef>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#if defined(__USE_POSIX)
#include <endian.h>
#endif

#if defined(__GNUC__) && (__GNUC__ < 4 || ( __GNUC__ == 4 && __GNUC_MINOR__ < 8))
#define __NONE_BUILDIN_BYTESWAP__
#endif

#ifndef __BYTE_ORDER
#if defined(__BYTE_ORDER__)
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define __BYTE_ORDER __BIG_ENDIAN
#else
#define __BYTE_ORDER __LITTLE_ENDIAN
#endif
#else

#if defined(__ARM_ARCH__)
#if defined(__BIG_ENDIAN)
#define __BYTE_ORDER __BIG_ENDIAN
#else
#define __BYTE_ORDER __LITTLE_ENDIAN
#endif
#else
#define __BYTE_ORDER __LITTLE_ENDIAN
#endif
#endif
#endif

/// 大端字节序定义
#ifndef __BIG_ENDIAN
#define __BIG_ENDIAN 4321
#endif

/// 小端字节序定义
#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN 1234
#endif

#ifndef __PDP_ENDIAN
#define __PDP_ENDIAN 3412
#endif

namespace ss {
namespace utils {

#ifdef __NONE_BUILDIN_BYTESWAP__
inline uint16_t __builtin_bswap16(uint16_t v)
{
    return ((v << 8u) | (v >> 8u));
}

inline uint32_t __builtin_bswap32(uint32_t v)
{
    return (v << 24u) | ((v << 8u) & 0x00ff0000u) | ((v >> 8u) & 0x0000ff00u) | (v >> 24u);
}

inline uint64_t __builtin_bswap64(uint64_t v)
{
    return v << 56u | v >> 56u
        | ((v << 40u) & 0x00ff000000000000ull) | ((v >> 40u) & 0x000000000000ff00ull)
        | ((v << 24u) & 0x0000ff0000000000ull) | ((v >> 24u) & 0x0000000000ff0000ul)
        | ((v << 8u ) & 0x000000ff00000000ull) | ((v >> 8u ) & 0x00000000ff000000ull);
}
#endif

/**
 * 8位无符号整数字节序翻转
 * @param v 要转换的数据
 * @return 返回翻转结果，实际上8位数据翻转字节序时什么都不做，输入和输出相等
 */
inline static uint8_t byteswap_uint8(uint8_t v)
{
    return v;
}

/**
 * 8位有符号整数字节序翻转
 * @param v 要转换的数据
 * @return 返回翻转结果，实际上8位数据翻转字节序时什么都不做，输入和输出相等
 */
inline static int8_t byteswap_int8(int8_t v)
{
    return v;
}

/**
 * 16位无符号整数字节序翻转
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static uint16_t byteswap_uint16(uint16_t v)
{
#if defined(_MSC_VER)
#pragma push_macro("__builtin_bswap16")
#define __builtin_bswap16 _byteswap_ushort
#endif
    return __builtin_bswap16(v);
#if defined(_MSC_VER)
#undef __builtin_bswap16
#pragma pop_macro("__builtin_bswap16")
#endif
}

/**
 * 16位有符号整数字节序翻转
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static int16_t byteswap_int16(int16_t v)
{
#if defined(_MSC_VER)
#pragma push_macro("__builtin_bswap16")
#define __builtin_bswap16 _byteswap_ushort
#endif
    uint16_t r = __builtin_bswap16(*reinterpret_cast<uint16_t *>(&v));
    return *reinterpret_cast<int16_t *>(&r);
#if defined(_MSC_VER)
#pragma pop_macro("__builtin_bswap16")
#endif
}

/**
 * 32位无符号整数字节序翻转
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static uint32_t byteswap_uint32(uint32_t v)
{
#if defined(_MSC_VER)
#pragma push_macro("__builtin_bswap32")
#define __builtin_bswap32 _byteswap_ulong
#endif
    return __builtin_bswap32(v);
#if defined(_MSC_VER)
#pragma pop_macro("__builtin_bswap32")
#endif
}

/**
 * 32位有符号整数字节序翻转
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static int32_t byteswap_int32(int32_t v)
{
#if defined(_MSC_VER)
#pragma push_macro("__builtin_bswap32")
#define __builtin_bswap32 _byteswap_ulong
#endif
    uint32_t r = __builtin_bswap32(*reinterpret_cast<uint32_t *>(&v));
    return *reinterpret_cast<int32_t *>(&r);
#if defined(_MSC_VER)
#pragma pop_macro("__builtin_bswap32")
#endif
}

/**
 * 64位无符号整数字节序翻转
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static uint64_t byteswap_uint64(uint64_t v)
{
#if defined(_MSC_VER)
#pragma push_macro("__builtin_bswap64")
#define __builtin_bswap64 _byteswap_uint64
#endif
    return __builtin_bswap64(v);
#if defined(_MSC_VER)
#pragma pop_macro("__builtin_bswap64")
#endif
}

/**
 * 64位有符号整数字节序翻转
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static int64_t byteswap_int64(int64_t v)
{
#if defined(_MSC_VER)
#pragma push_macro("__builtin_bswap64")
#define __builtin_bswap64 _byteswap_uint64
#endif
    uint64_t r = __builtin_bswap64(*reinterpret_cast<uint64_t *>(&v));
    return *reinterpret_cast<int64_t *>(&r);
#if defined(_MSC_VER)
#pragma pop_macro("__builtin_bswap64")
#endif
}

/**
 * 8位有符号整数字节序翻转，等同于@see byteswap_int8()
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static int8_t byteswap(int8_t v)
{
    return byteswap_int8(v);
}

/**
 * 8位无符号整数字节序翻转，等同于@see byteswap_uint8()
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static uint8_t byteswap(uint8_t v)
{
    return byteswap_uint8(v);
}

/**
 * 16位有符号整数字节序翻转，等同于@see byteswap_int16()
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static int16_t byteswap(int16_t v)
{
    return byteswap_int16(v);
}

/**
 * 16位无符号整数字节序翻转，等同于@see byteswap_uint16()
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static uint16_t byteswap(uint16_t v)
{
    return byteswap_uint16(v);
}

/**
 * 32位有符号整数字节序翻转，等同于@see byteswap_int32()
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static int32_t byteswap(int32_t v)
{
    return byteswap_int32(v);
}

/**
 * 32位无符号整数字节序翻转，等同于@see byteswap_uint32()
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static uint32_t byteswap(uint32_t v)
{
    return byteswap_uint32(v);
}

/**
 * 64位有符号整数字节序翻转，等同于@see byteswap_int64()
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static int64_t byteswap(int64_t v)
{
    return byteswap_int64(v);
}

/**
 * 64位无符号整数字节序翻转，等同于@see byteswap_uint64()
 * @param v 要转换的数据
 * @return 返回翻转结果
 */
inline static uint64_t byteswap(uint64_t v)
{
    return byteswap_uint64(v);
}

#if defined(__GNUC__) && (__WORDSIZE == 64) && !defined(__clang__)
inline static int64_t byteswap(long long int v)
{
    return byteswap_int64(v);
}

inline static uint64_t byteswap(unsigned long long int v)
{
    return byteswap_uint64(v);
}
#endif

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define __NETWORK_BYTE_SWAP__ 1
#else
#define __NETWORK_BYTE_SWAP__ 0
#endif

#if __NETWORK_BYTE_SWAP__
#define byteswap_net(v)         byteswap(v)
#define byteswap_net_int8(v)    byteswap_int8(v)
#define byteswap_net_uint8(v)   byteswap_uint8(v)
#define byteswap_net_int16(v)   byteswap_int16(v)
#define byteswap_net_uint16(v)  byteswap_uint16(v)
#define byteswap_net_int32(v)   byteswap_int32(v)
#define byteswap_net_uint32(v)  byteswap_uint32(v)
#define byteswap_net_int64(v)   byteswap_int64(v)
#define byteswap_net_uint64(v)  byteswap_uint64(v)
#else
#define byteswap_net(v)        (v)
#define byteswap_net_int8(v)   (v)
#define byteswap_net_uint8(v)  (v)
#define byteswap_net_int16(v)  (v)
#define byteswap_net_uint16(v) (v)
#define byteswap_net_int32(v)  (v)
#define byteswap_net_uint32(v) (v)
#define byteswap_net_int64(v)  (v)
#define byteswap_net_uint64(v) (v)
#endif

}
}

#endif //__STAR_SDK_UTILS_BYTES_ORDER_H
