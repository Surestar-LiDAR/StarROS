/**
 * @author xiaoma 
 * @date 2019/12/2.
 */

#ifndef __STAR_SDK_UTILS_DETAIL_BUFFER_H
#define __STAR_SDK_UTILS_DETAIL_BUFFER_H

#include <cstddef>
#include <type_traits>
#include <cstring>
#include <algorithm>

namespace ss {
namespace utils {
namespace detail {

template<typename _Out, typename _In,
        typename std::enable_if<std::is_pod<_Out>::value && std::is_pod<_In>::value, int>::type = 0>
inline void __do_memcpy(_Out* __result, _In* __from, std::size_t __length)
{
    __length = __length * sizeof(_In);
    ::memcpy(__result, __from, __length);
}

template<typename _Out, typename _In,
        typename std::enable_if<!std::is_pod<_Out>::value || !std::is_pod<_In>::value, int>::type = 0>
inline void __do_memcpy(_Out* __result, _In* __from, std::size_t __length)
{
    std::copy(__from, __from + __length, __result);
}

template<typename _Out, typename _In>
inline void memcpy(_Out* __result, _In* __from, std::size_t __length)
{
    return __do_memcpy(__result, __from, __length);
}

}
}
}

#endif //__STAR_SDK_UTILS_DETAIL_BUFFER_H
