/**
 * @author   lucb
 * @date     2019/12/10
 */

#ifndef __STAR_SDK_BASIC_MESSAGE_H
#define __STAR_SDK_BASIC_MESSAGE_H

#include <star/msg/Message.h>

#include <star/msg//detail/encode.h>

#include <cstring>

namespace ss {
namespace msg {
namespace detail {

template <typename _Byte, typename _DataType>
class basic_message : public Message {
public:
    typedef _DataType data_type;
    inline ssize_t decode(ss::utils::buffer_reader& __reader) override
    {
        return detail::decode<_Byte>(__reader, &_data);
    }

    inline ssize_t encode(ss::utils::buffer_writer& __writer) override
    {
        return detail::encode<_Byte>(__writer, &_data);
    }

    ssize_t dump(void* storage, size_t length) const override
    {
        if(length < sizeof(_data)) {
            return 0;
        }
        memmove(storage, &_data, sizeof(_data));
        return sizeof(_data);
    }

    size_t size() override
    {
        return sizeof(data_type);
    }

    data_type& data()
    {
        return _data;
    }

    const data_type& data() const
    {
        return _data;
    }

    void set_data(const data_type& data)
    {
        _data = data;
    }

	static std::size_t min_size()
    {
        return sizeof(data_type);
    }

    static std::size_t max_size()
    {
        return sizeof(data_type);
    }
private:
    data_type _data;
};

}
}
}


#endif //__STAR_SDK_BASIC_MESSAGE_H
