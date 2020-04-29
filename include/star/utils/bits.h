/**
 * @author   lucb
 * @date     2020/1/8
 */

#ifndef __STAR_SDK_UTILS_BITS_H
#define __STAR_SDK_UTILS_BITS_H

namespace ss {
namespace utils {

template <typename _Ret, size_t start, size_t stop, typename _Byte>
_Ret extract(_Byte byte)
{
    static_assert(start <= stop, "star must smaller than stop.");
    return static_cast<_Ret>((byte >> start) & ((1u << (stop - start + 1)) - 1u));
}

template <typename _Ret, size_t pos1, size_t pos2, typename _Byte1, typename _Byte2>
_Ret splice(_Byte1 byte1, _Byte2 byte2)
{
    return (static_cast<_Ret>(byte1) << pos1) | (static_cast<_Ret>(byte2) << pos2);
}

template <typename _Ret, size_t pos1, size_t pos2, size_t pos3, typename _Byte1, typename _Byte2, typename _Byte3>
_Ret splice(_Byte1 byte1, _Byte2 byte2, _Byte3 byte3)
{
    return (static_cast<_Ret>(byte1) << pos1) | (static_cast<_Ret>(byte2) << pos2) | (static_cast<_Ret>(byte3) << pos3);
}

template <typename _Ret, size_t pos1, size_t pos2, size_t pos3, size_t pos4, typename _Byte1, typename _Byte2, typename _Byte3, typename _Byte4>
_Ret splice(_Byte1 byte1, _Byte2 byte2, _Byte3 byte3, _Byte4 byte4)
{
    return (static_cast<_Ret>(byte1) << pos1) | (static_cast<_Ret>(byte2) << pos2) | (static_cast<_Ret>(byte3) << pos3) | (static_cast<_Ret>(byte4) << pos4);
}

}
}

#endif //__STAR_SDK_UTILS_BITS_H
