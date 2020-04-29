/**
 * @author   lucb
 * @date     2019/12/10
 */

#ifndef __STAR_SDK_BASIC_WORD_MESSAGE_H
#define __STAR_SDK_BASIC_WORD_MESSAGE_H

#include <star/Star.h>

namespace ss {
namespace msg {
namespace detail {

template <typename _Byte, typename _Data, typename _Reader>
ssize_t decode (_Reader& __reader, _Data* __data)
{
    if(__reader.size() < sizeof(_Data)) {
        return -1;
    }

    try {
        return __reader.get_value(reinterpret_cast<_Byte*>(__data), sizeof(_Data) / sizeof(_Byte)) * sizeof(_Byte);
    }
    catch (const std::exception&) {
        return -1;
    }
}

template <typename _Byte, typename _Data, typename _Writer>
ssize_t encode (_Writer& __writer, const _Data* __data)
{
    if(__writer.size() < sizeof(_Data)) {
        return -1;
    }

    try {
        return __writer.put_value(reinterpret_cast<const _Byte*>(__data), sizeof(_Data) / sizeof(_Byte)) * sizeof(_Byte);
    }
    catch (const std::exception&) {
        return -1;
    }
}

}
}
}

#endif //__STAR_SDK_BASIC_WORD_MESSAGE_H
