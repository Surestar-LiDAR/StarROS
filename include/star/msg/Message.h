/**
 * @author   lucb
 * @date     2019/12/9
 */

#ifndef __STAR_SDK_MESSAGE_H
#define __STAR_SDK_MESSAGE_H

#include <star/Star.h>

#include <star/utils/buffer_reader.h>
#include <star/utils/buffer_writer.h>

#if defined(_MSC_VER)
#else
#endif

namespace ss {
namespace msg {

class __star_export Message {
public:
    virtual ~Message() = default;
    virtual uint64_t dataId() const = 0;
    virtual ssize_t decode(ss::utils::buffer_reader& __reader) = 0;
    virtual ssize_t encode(ss::utils::buffer_writer& __writer) = 0;
    virtual ssize_t dump(void* storage, size_t length) const = 0;
    virtual size_t size() = 0;
};

}
}


#endif //__STAR_SDK_MESSAGE_H
