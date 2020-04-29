/**
 * @author      : John 
 * @date        : 2018-10-12
 */

#ifndef __PHOTON_BUFFER_WRITER_H
#define __PHOTON_BUFFER_WRITER_H

#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <type_traits>

#include <star/utils/bytes_order.h>
#include <star/utils/detail/buffer_traits.h>

namespace ss {
namespace utils {
namespace detail {

template<typename _Buffer>
struct buffer_write_helper
{
public:
    typedef typename buffer_traits<_Buffer>::item_type item_type;

    template<typename _Value>
    inline static std::size_t put_value(_Buffer& buffer, const _Value* value, std::size_t count)
    {
        std::size_t length = count * sizeof(_Value) / sizeof(item_type);
        buffer.write(reinterpret_cast<const item_type*>(value), length);
        return count;
    }
};


template<typename _Buffer>
struct byteswap_buffer_write_helper
{
public:
    typedef typename buffer_traits<_Buffer>::item_type item_type;

    template<typename _Value, typename std::enable_if<(sizeof(_Value) == 1), int>::type = 0>
    inline static std::size_t put_value(_Buffer& buffer, const _Value* value, std::size_t count)
    {
        std::size_t length = count * sizeof(_Value) / sizeof(item_type);
        buffer.write(reinterpret_cast<const item_type*>(value), length);
        return count;
    }

    template<typename _Value, typename std::enable_if<(sizeof(_Value) > 1), int>::type = 0>
    inline static std::size_t put_value(_Buffer& buffer, const _Value* value, std::size_t count)
    {
        std::size_t item_length = sizeof(_Value) / sizeof(item_type);

        std::size_t length = count * item_length;
        if(buffer.available() < length) {
            throw std::out_of_range("buffer::write()");
        }

        _Value temp;
        for(std::size_t idx = 0; idx < count; ++idx) {
            temp = byteswap(value[idx]);
            buffer.write(reinterpret_cast<const item_type*>(&temp), item_length);
        }
        return count;
    }

    template<typename _Value>
    inline static _Value byteswap(_Value value)
    {
        return utils::byteswap(value);
    }

    inline static float byteswap(float value)
    {
        return value;
    }

    inline static double byteswap(double value)
    {
        return value;
    }
};


template<typename _Buffer, bool _Network = true>
struct buffer_writer_traits
{
public:
    using item_type = typename buffer_traits<_Buffer>::item_type;
    using write_helper = typename std::conditional<_Network && (__BYTE_ORDER == __LITTLE_ENDIAN),
                                   byteswap_buffer_write_helper<_Buffer>,
                                   buffer_write_helper<_Buffer>>::type;

    template<typename _Value>
    inline static std::size_t put_value(_Buffer &buffer, const _Value *value, std::size_t count)
    {
        return write_helper::put_value(buffer, value, count);
    }

    inline static std::size_t tellp(_Buffer& buffer) noexcept
    {
        return buffer.tellp();
    }

    inline static void seekp(_Buffer& buffer, std::size_t offset) noexcept
    {
        buffer.seekp(offset);
    }

    inline static void sync(_Buffer& buffer) noexcept
    {
        buffer.sync();
    }

    inline static std::size_t size(_Buffer& buffer) noexcept
    {
        return buffer.available();
    }
};

} //detail

class buffer_writer {
public:
//    virtual std::size_t write(const uint8_t* data, std::size_t length) = 0;
    virtual ~buffer_writer() noexcept = default;
	
    virtual std::size_t tellp() const = 0;

    virtual void seekp(std::size_t offset) = 0;

    virtual std::size_t size() const noexcept = 0;

    virtual std::size_t available() const noexcept = 0;
    
    inline void put_int8(int8_t value)
    {
        put_uint8(reinterpret_cast<const uint8_t*>(&value), 1u);
    }

    inline void put_int8(uint8_t value)
    {
        put_uint8(&value, 1u);
    }

    inline void put_int16(int16_t value)
    {
        put_uint16(reinterpret_cast<const uint16_t*>(&value), 1u);
    }

    inline void put_int16(uint16_t value)
    {
        put_uint16(&value, 1u);
    }

    inline void put_int32(int32_t value)
    {
        put_uint32(reinterpret_cast<const uint32_t*>(&value), 1u);
    }

    inline void put_int32(uint32_t value)
    {
        put_uint32(&value, 1u);
    }

    inline void put_int64(int64_t value)
    {
        put_uint64(reinterpret_cast<const uint64_t*>(&value), 1u);
    }

    inline void put_int64(uint64_t value)
    {
        put_uint64(&value, 1u);
    }

    inline void put_float(float value)
    {
        put_float(&value, 1u);
    }

    inline void put_double(double value)
    {
        put_double(&value, 1u);
    }

    virtual std::size_t put_uint8(const uint8_t* value, std::size_t length) = 0;
    virtual std::size_t put_uint16(const uint16_t* value, std::size_t length) = 0;
    virtual std::size_t put_uint32(const uint32_t* value, std::size_t length) = 0;
    virtual std::size_t put_uint64(const uint64_t* value, std::size_t length) = 0;
    virtual std::size_t put_float(const float* value, std::size_t length) = 0;
    virtual std::size_t put_double(const double* value, std::size_t length) = 0;

    inline std::size_t put_int8(const int8_t* value, std::size_t length)
    {
        return this->put_uint8(reinterpret_cast<const uint8_t*>(value), length);
    }

    inline std::size_t put_int16(const int16_t* value, std::size_t length)
    {
        return this->put_uint16(reinterpret_cast<const uint16_t*>(value), length);
    }

    inline std::size_t put_int32(const int32_t* value, std::size_t length)
    {
        return this->put_uint32(reinterpret_cast<const uint32_t*>(value), length);
    }

    inline std::size_t put_int64(const int64_t* value, std::size_t length)
    {
        return this->put_uint64(reinterpret_cast<const uint64_t*>(value), length);
    }

    template<typename _Value>
    inline void put_value(const _Value& value)
    {
        this->do_put_value(&value, 1u);
    }

    template<typename _Value>
    inline std::size_t put_value(const _Value* value, std::size_t length)
    {
        return this->do_put_value(value, length);
    }

private:
    template <typename _Tp, typename std::enable_if<std::is_integral<_Tp>::value, int>::type = 0 >
    inline std::size_t do_put_value(const _Tp* value, std::size_t length)
    {
        return this->do_put_value(reinterpret_cast<const typename std::make_unsigned<_Tp>::type *>(value), length);
    }

    inline std::size_t do_put_value(const int8_t* value, std::size_t length)
    {
        return this->put_int8(value, length);
    }

    inline std::size_t do_put_value(const uint8_t* value, std::size_t length)
    {
        return this->put_uint8(value, length);
    }

    inline std::size_t do_put_value(const int16_t* value, std::size_t length)
    {
        return this->put_int16(value, length);
    }

    inline std::size_t do_put_value(const uint16_t* value, std::size_t length)
    {
        return this->put_uint16(value, length);
    }

    inline std::size_t do_put_value(const int32_t* value, std::size_t length)
    {
        return this->put_int32(value, length);
    }

    inline std::size_t do_put_value(const uint32_t* value, std::size_t length)
    {
        return this->put_uint32(value, length);
    }

    inline std::size_t do_put_value(const int64_t* value, std::size_t length)
    {
        return this->put_int64(value, length);
    }

    inline std::size_t do_put_value(const uint64_t* value, std::size_t length)
    {
        return this->put_uint64(value, length);
    }

    inline std::size_t do_put_value(const float* value, std::size_t length)
    {
        return this->put_float(value, length);
    }

    inline std::size_t do_put_value(const double* value, std::size_t length)
    {
        return this->put_double(value, length);
    }

};

#if 0
template<typename _Buffer, typename _Traits>
class basic_buffer_writer {
public:
    using buffer_type = _Buffer;
    using traits_type = _Traits;

    explicit basic_buffer_writer(buffer_type& buffer) :
            _buffer(buffer)
    {
    }

    ~basic_buffer_writer()
    {
        this->sync();
    }

    template<typename _Value>
    inline std::size_t put_value(const _Value& value)
    {
        std::size_t size = traits_type::template put_value(_buffer, &value, 1);
        if (size == 0) {
            throw std::out_of_range("buffer_writer::put_value");
        }
        return size;
    }

    template<typename _Value>
    inline std::size_t put_value(const _Value* value, std::size_t length)
    {
        std::size_t size = traits_type::template put_value(_buffer, value, length);
        if (size == 0) {
            throw std::out_of_range("buffer_writer::put_value");
        }
        return length;
    }

    inline std::size_t put_int8(int8_t value)
    {
        return put_value<int8_t>(value);
    }

    inline std::size_t put_uint8(uint8_t value)
    {
        return put_value<uint8_t>(value);
    }

    inline std::size_t put_int16(int16_t value)
    {
        return put_value<int16_t>(value);
    }

    inline std::size_t put_uint16(uint16_t value)
    {
        return put_value<uint16_t>(value);
    }

    inline std::size_t put_int32(int32_t value)
    {
        return put_value<int32_t>(value);
    }

    inline std::size_t put_uint32(uint32_t value)
    {
        return put_value<uint32_t>(value);
    }

    inline std::size_t put_int64(int64_t value)
    {
        return put_value<int64_t>(value);
    }

    inline std::size_t put_uint64(uint64_t value)
    {
        return put_value<uint64_t>(value);
    }

    std::size_t tellp() const
    {
        return traits_type::tellp(_buffer);
    }

    void seekp(std::size_t offset)
    {
        traits_type::seekp(_buffer, offset);
    }

    void sync() noexcept
    {
        return traits_type::sync(_buffer);
    }

    inline std::size_t size() const noexcept
    {
        return traits_type::size(_buffer);
    }

    inline std::size_t available() const
    {
        return this->size();
    }

private:
    buffer_type&        _buffer;
};
#else
template<typename _Buffer, typename _Traits>
class basic_buffer_writer : public buffer_writer {
public:
    using buffer_type = _Buffer;
    using traits_type = _Traits;

    explicit basic_buffer_writer(buffer_type& buffer) :
        _buffer(buffer)
    {
    }

    ~basic_buffer_writer() noexcept
    {
        this->sync();
    }

    inline size_t put_uint8(const uint8_t* value, std::size_t length) override
    {
        return traits_type::put_value(_buffer, value, length);
    }

    inline size_t put_uint16(const uint16_t* value, std::size_t length) override
    {
        return traits_type::put_value(_buffer, value, length);
    }

    inline size_t put_uint32(const uint32_t* value, std::size_t length) override
    {
        return traits_type::put_value(_buffer, value, length);
    }

    inline size_t put_uint64(const uint64_t* value, std::size_t length) override
    {
        return traits_type::put_value(_buffer, value, length);
    }

    inline size_t put_float(const float* value, std::size_t length) override
    {
        return traits_type::put_value(_buffer, value, length);
    }

    inline size_t put_double(const double* value, std::size_t length) override
    {
        return traits_type::put_value(_buffer, value, length);
    }

    std::size_t tellp() const override
    {
        return traits_type::tellp(_buffer);
    }

    void seekp(std::size_t offset) override
    {
        traits_type::seekp(_buffer, offset);
    }

    void sync() noexcept
    {
        return traits_type::sync(_buffer);
    }

    inline std::size_t size() const noexcept override
    {
        return traits_type::size(_buffer);
    }

    inline std::size_t available() const noexcept override
    {
        return this->size();
    }

    buffer_type& buf()
    {
        return _buffer;
    }

    const buffer_type& buf() const
    {
        return _buffer;
    }

private:
    buffer_type&        _buffer;
};
#endif

template<typename _Buffer>
using native_buffer_writer = basic_buffer_writer<_Buffer, detail::buffer_writer_traits<_Buffer, false>>;

template<typename _Buffer>
using network_buffer_writer = basic_buffer_writer<_Buffer, detail::buffer_writer_traits<_Buffer, true>>;

}
}

#endif //__PHOTON_BUFFER_WRITER_H
