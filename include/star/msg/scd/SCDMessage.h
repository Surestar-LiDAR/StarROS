/**
 * @author   lucb
 * @date     2019/12/12
 */

#ifndef __STAR_SDK_SCD_MESSAGE_H
#define __STAR_SDK_SCD_MESSAGE_H

#include <star/msg/detail/basic_message.h>

namespace ss {
namespace msg {
namespace scd {

template <typename _DataType>
class Message : public detail::basic_message<uint16_t, _DataType>{
public:
};

}

}
}
#endif //__STAR_SDK_SCD_MESSAGE_H
