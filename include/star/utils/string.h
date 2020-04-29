/**
 * @date        : 2017-03-21
 * @version     : 1.0.0
 * @brief       :
 */
#ifndef __STAR_SDK_UTILS_STRINGS_H
#define __STAR_SDK_UTILS_STRINGS_H

#include <cstring>

namespace utils {

    template<typename _String>
    struct string_traits;

    template<template<typename X> class _String, typename _Char>
    struct string_traits<_String<_Char> >
    {
        typedef _Char char_type;
    };

    template<template<typename X, typename A> class _String, typename _Char, typename _Allocator>
    struct string_traits<_String<_Char, _Allocator> >
    {
        typedef _Char char_type;
    };

    template<template<typename X, typename T, typename A> class _String, typename _Char, typename _Traits, typename _Allocator>
    struct string_traits<_String<_Char, _Traits, _Allocator> >
    {
        typedef _Char char_type;
    };

    template<typename _Char, typename _Traits, typename _Allocator>
    struct string_traits<std::basic_string<_Char, _Traits, _Allocator> >
    {
        typedef _Char char_type;
    };

    template<>
    struct string_traits<std::string>
    {
        typedef char char_type;
    };

    template<>
    struct string_traits<std::wstring>
    {
        typedef wchar_t char_type;
    };

    namespace detail {

        /**
         * 通配符匹配
         * @param pattern        匹配模式，*代表任意字符（包含空）?代表一个字符
         * @param patternLength  匹配模式长度
         * @param content        匹配内容
         * @param contentLength  匹配内容长度
         * @param first          *标记
         * @return 匹配成功则返回true，否则返回false
         */
        template <typename _Char>
        bool strmatch(const _Char pattern[], size_t patternLength, const _Char content[], size_t contentLength, bool first = true)
        {
            size_t patternOffset = 0;
            size_t contentOffset = 0;

            while (true) {
                if(contentOffset == contentLength || content[contentOffset] == '\0') {
                    return patternOffset == patternLength;
                }

                if (pattern[patternOffset] == '*') { //递归比较
                    ++patternOffset;

                    if(patternOffset == patternLength || pattern[patternOffset] == '\0') { //后面的内容可以不不再比较
                        return true;
                    }

                    return (strmatch<_Char>(pattern + patternOffset,
                            patternLength - patternOffset,
                            content + contentOffset,
                            contentLength - contentOffset, false));
                }

                if (pattern[patternOffset] == '?') { //匹配？直接忽略这个字符
                    ++patternOffset;
                    ++contentOffset;
                }

                else if (pattern[patternOffset] == content[contentOffset]) {
                    ++patternOffset;
                    ++contentOffset;
                }
                else {
                    if (!first) { //匹配*后面的内容失败，从头匹配
                        patternOffset = 0;
                        ++contentOffset;
                    }
                    else {
                        return false;
                    }
                }
            }
        }
    }

    /**
     * 通配符匹配
     * @param pattern        匹配模式，*代表任意字符（包含空）?代表一个字符
     * @param patternLength  匹配模式长度
     * @param content        匹配内容
     * @param contentLength  匹配内容长度
     * @return 匹配成功则返回true，否则返回false
     */
    template <typename _Char>
    bool strmatch(const _Char pattern[], size_t patternLength, const _Char content[], size_t contentLength)
    {
        return detail::strmatch(pattern, patternLength, content, contentLength, true);
    }

    /**
     *
     * @tparam _String
     * @param pattern  匹配模式，*代表任意字符（包含空）?代表一个字符
     * @param content  匹配内容
     * @return 匹配成功则返回true，否则返回false
     */
    template <typename _String>
    bool strmatch(const _String &pattern, const _String &content)
    {
        return detail::strmatch(pattern.data(), pattern.length(), content.data(), content.length, true);
    }

    /**
     * 替换字符串中的指定字符
     * @tparam _String 字符串类型
     * @param source 字符串
     * @param target 字符传中的指定字符
     * @param dist   替换的字符串
     * @return
     */
    template <typename _String>
    size_t strexchang(_String &source, typename string_traits<_String>::char_type target, typename string_traits<_String>::char_type dist)
    {
        size_t count = 0;
        for(size_t idx = 0; idx < source.length(); ++idx) {
            if(source[idx] == target) {
                source[idx] = dist;
                ++count;
            }
        }
        return count;
    }

    /**
     * 替换字符串中的指定字符
     * @tparam _Char 字符数组类型
     * @param source 字符串
     * @param length 字符串长度
     * @param target 字符传中的指定字符
     * @param dist 替换的字符串
     * @return
     */
    template <typename _Char>
    size_t strexchang(_Char *source, size_t length, _Char target, _Char dist)
    {
        size_t count = 0;
        for(size_t idx = 0; idx < length && source[idx] != '\0'; ++idx) {
            if(source[idx] == target) {
                ++count;
                source[idx] = dist;
            }
        }
        return count;
    }

    template <typename _String>
    _String strreplace(const _String &source, typename string_traits<_String>::char_type target, const _String &dist)
    {
        _String ret;
        for(size_t idx = 0; idx < source.length(); ++idx) {
            if(source[idx] == target) {
                ret += dist;
            }
            else {
                ret += source[idx];
            }
        }
        return ret;
    }

    template <typename _String>
    _String strreplace(const _String &source, typename string_traits<_String>::char_type target, const typename string_traits<_String>::char_type *dist)
    {
        _String ret;
        for(size_t idx = 0; idx < source.length(); ++idx) {
            if(source[idx] == target) {
                ret += dist;
            }
            else {
                ret += source[idx];
            }
        }
        return ret;
    }

    template <typename _Char, typename _Set>
    ssize_t find_first(_Char *source, size_t length, _Char target, size_t offset = 0)
    {
        for(size_t idx = offset; idx < length; ++idx) {
            if(source[idx] == target) {
                return (ssize_t)idx;
            }
        }
        return -1;
    }

    template <typename _Char, typename _Set>
    ssize_t find_first(_Char *source, size_t length, const _Set &set, size_t offset = 0)
    {
        for(size_t idx = offset; idx < length; ++idx) {
            if(set.find(source[idx]) != set.end()) {
                return (ssize_t)idx;
            }
        }
        return -1;
    }

    template <typename _Char, typename _Set>
    ssize_t find_first_not(_Char *source, size_t length, const _Char target, size_t offset = 0)
    {
        for(size_t idx = offset; idx < length; ++idx) {
            if(source[idx] != target) {
                return (ssize_t)idx;
            }
        }
        return -1;
    }

    template <typename _Char, typename _Set>
    ssize_t find_first_not(_Char *source, size_t length, const _Set &set, size_t offset = 0)
    {
        for(size_t idx = offset; idx < length; ++idx) {
            if(set.find(source[idx]) == set.end()) {
                return (ssize_t)idx;
            }
        }
        return -1;
    }

//    template <typename _Char, typename _Tp>
//    inline size_t integral_to_string(_Char *buffer, size_t length, _Tp value)
//    {
//        if (length == 0) {
//            return 0;
//        }
//        if(value < 0) {
//            *(buffer++) = '-';
//            return internal::__integral_to_string(buffer, length - 1, static_cast<typename __make_integer<sizeof(_Tp)>::unsigned_type>(~value + 1)) + 1;
//        }
//
//        return internal::__integral_to_string(buffer, length, static_cast<typename __make_integer<sizeof(_Tp)>::unsigned_type>(value));
//    }

#if 0
    template <typename _Char, typename _Tp>
    inline size_t floating_to_string(_Char *buffer, size_t length, _Tp value)
    {
        return internal::__floating_to_string(buffer, length, value);
    }
#endif

    template <typename _Tp>
    inline std::string to_string(_Tp val)
    {
        char buffer[4 * sizeof(_Tp)];
        size_t length = integral_to_string(buffer, 4 * sizeof(unsigned long long), val);
        return std::string(buffer, length);
    }

    template <typename _Tp>
    inline std::wstring to_wstring(_Tp val)
    {
        wchar_t buffer[4 * sizeof(_Tp)];
        size_t length = integral_to_string(buffer, 4 * sizeof(unsigned long long), val);
        return std::wstring(buffer, length);
    }

#if 0
    template <>
    inline std::string to_string<float>(float val)
    {
        char buffer[38 + 20];
        size_t length = floating_to_string(buffer, 38 + 20, val);
        return std::string(buffer, length);
    }

    template <>
    inline std::string to_string<double>(double val)
    {
        char buffer[308 + 20];
        size_t length = floating_to_string(buffer, 308 + 20, val);
        return std::string(buffer, length);
    }

    template <>
    inline std::string to_string<long double>(long double val)
    {
        char buffer[4932 + 20];
        size_t length = floating_to_string(buffer, 4932 + 20, val);
        return std::string(buffer, length);
    }

    inline std::wstring to_wstring(float val)
    {
        wchar_t buffer[38 + 20]; //gcc __FLT_MAX_10_EXP__ + 20
        size_t length = floating_to_string(buffer, 38 + 20, val);
        return std::wstring(buffer, length);
    }

    inline std::wstring to_wstring(double val)
    {
        wchar_t buffer[308 + 20]; //gcc __DBL_MAX_10_EXP__ + 20 //todo better buffer
        size_t length = floating_to_string(buffer, 308 + 20, val);
        return std::wstring(buffer, length);
    }

    inline std::wstring to_wstring(long double val)
    {
        wchar_t buffer[4932 + 20]; //gcc __LDBL_MAX_10_EXP__ + 20 //todo better buffer
        size_t length = floating_to_string(buffer, 4932 + 20, val);
        return std::wstring(buffer, length);
    }
#endif

    template <typename _Char>
    inline size_t hex2str(const char *hex, size_t hex_len, _Char *str, size_t str_len, bool upper = false)
    {
        const static _Char __hex_small[] = {
            '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
        };
        const static _Char __hex_upper[] = {
            '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
        };
        size_t idx;

        const _Char *__hex = upper ? __hex_upper : __hex_small;

        for(idx = 0; idx < hex_len; ++idx) {
            size_t off = idx << 1;
            if(off >= str_len) {
                str[str_len - 1] = 0;
                return str_len - 1;
            }
            str[off + 1] = __hex[hex[idx] & 0x0f];
            str[off    ] = __hex[(hex[idx] >> 4) & 0x0f];
        }

        str[idx << 1] = 0;
        return idx << 1;
    }

    namespace detail {
        template<typename _Char>
        inline char __hex(_Char c)
        {
            const static _Char A = 'A';
            const static _Char F = 'F';
            const static _Char a = 'a';
            const static _Char f = 'f';
            const static _Char zero = '0';
            const static _Char nine = '9';

            if(c >= zero && c <= nine) {
                return c - zero;
            }
            else if(c >= A && c <= F) {
                return c - A + (_Char)0x0A;
            }
            else if(c >= a && c <= f) {
                return c - a + (_Char)0x0a;
            }
            return 0;
        }
    }

    template <typename _Char>
    inline size_t str2hex(const _Char *str, size_t str_len, char *hex, size_t hex_len)
    {
        size_t idx;
        for(idx = 0; idx < hex_len; ++idx) {
            size_t off = idx << 1;
            if(off >= str_len) {
                break;
            }
            hex[idx] = detail::__hex(str[off + 1]) | (detail::__hex(str[off]) << 4) ;
        }

        return idx;
    }

#if 0
    template <typename _Char>
    inline _Char *lowercase(_Char *str, size_t length)
    {
        const static _Char A = 'A';
        const static _Char Z = 'Z';
        const static _Char convert = 'A' - 'a';
        for(size_t idx = 0; idx < length; ++idx) {
            if(str[idx] >= A && str[idx] <= Z) {
                str[idx] -= convert;
            }
        }
        return str;
    }
#endif

    template <typename _Char>
    inline size_t strlen(const _Char *str)
    {
        size_t idx = 0;
        while(str[idx] == '\0') {
            ++idx;
        }
        return idx;
    }

    template <>
    inline size_t strlen<char>(const char *str)
    {
        return ::strlen(str);
    }

    template <>
    inline size_t strlen<wchar_t>(const wchar_t *str)
    {
        return ::wcslen(str);
    }

    template <typename _Char>
    inline size_t strnlen(const _Char *str, size_t length)
    {
        size_t idx = 0;
        while(str[idx] == '\0' && idx != length) {
            ++idx;
        }
        return idx;
    }

    template <>
    inline size_t strnlen<char>(const char *str, size_t length)
    {
        return ::strnlen(str, length);
    }

    template <>
    inline size_t strnlen<wchar_t>(const wchar_t *str, size_t length)
    {
        return ::wcsnlen(str, length);
    }

    template <typename _Char>
    inline int strncmp(const _Char *str1, const _Char *str2, size_t length)
    {
        for(size_t idx = 0; idx < length; ++idx) {
            char ret = str1[idx] - str2[idx];
            if(ret != 0) {
                return ret;
            }
        }
        return 0;
    };

    template <>
    inline int strncmp<char>(const char *str1, const char *str2, size_t length)
    {
        return ::strncmp(str1, str2, length);
    };

    template <>
    inline int strncmp<wchar_t>(const wchar_t *str1, const wchar_t *str2, size_t length)
    {
        return ::wcsncmp(str1, str2, length);
    };
}

#endif //__STAR_SDK_UTILS_STRINGS_H
