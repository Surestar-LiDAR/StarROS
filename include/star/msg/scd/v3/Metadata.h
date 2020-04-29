/**
 * @author   lucb
 * @date     2020/1/14
 */

#ifndef __STAR_SDK_MSG_SCD_V3_META_DATA_H
#define __STAR_SDK_MSG_SCD_V3_META_DATA_H

#include <cstdint>
#include <star/msg/detail/basic_message.h>

namespace ss {
namespace msg {
namespace scd {
namespace v3 {

union Register {
    float    f;
    uint32_t i;
};

struct MetaData_s {
    uint32_t version;
    Register registers[65536];
};

class Metadata : public detail::basic_message<uint16_t, MetaData_s> {
public:
uint64_t dataId() const override;
};

}
}
}
}

#endif //__STAR_SDK_MSG_SCD_V3_META_DATA_H
