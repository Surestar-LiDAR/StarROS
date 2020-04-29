/**
 * @author      : John 
 * @date        : 2018-10-12
 */

#ifndef __STAR_SDK_UTILS_BUFFER_READER_H
#define __STAR_SDK_UTILS_BUFFER_READER_H

#include <cstdint>
#include <cstddef>
#include <stdexcept>

#include <star/utils/detail/buffer_traits.h>
#include <star/utils/bytes_order.h>

namespace ss {
namespace utils {
namespace detail {

template <typename _Buffer, bool _isByte>
struct buffer_read_value {

    template <typename _Value>
    inline static _Value get(_Buffer& buffer)
    {
        typedef typename buffer_traits<_Buffer>::item_type item_type;
        _Value value;
        std::size_t length = sizeof(_Value) / sizeof(item_type);
        buffer.read(reinterpret_cast<item_type*>(&value), length);
        return value;
    }

    template <typename _Value>
    inline static _Value byteswap_get(_Buffer& buffer)
    {
        typedef typename buffer_traits<_Buffer>::item_type item_type;
        _Value value;
        std::size_t length = sizeof(_Value) / sizeof(item_type);
        buffer.read(reinterpret_cast<item_type*>(&value), length);
        return  byteswap(value);
    }
};

#if 0
template <typename _Buffer>
struct buffer_read_value<_Buffer, true> {
private:
    template <typename _Value>
    struct value_traits {};
public:
    typedef typename buffer_traits<_Buffer>::item_type item_type;

    template <typename _Value>
    inline static _Value get(_Buffer& buffer)
    {
        if(buffer.size() < sizeof(_Value)) {
            throw std::out_of_range("buffer::get()");
        }
        _Value value = __do_get(buffer, value_traits<_Value>());

        buffer.seekg(buffer.tellg() + sizeof(_Value));
        return value;
    }

private:
    template <typename _Value>
    inline static _Value __do_get(_Buffer& buffer, value_traits<_Value>)
    {
        return 0;
    }

    inline static int8_t __do_get(_Buffer& buffer, value_traits<int8_t>)
    {
        item_type value = buffer[0];
        return *reinterpret_cast<int8_t*>(&value);
    }

    inline static uint8_t __do_get(_Buffer& buffer, value_traits<uint8_t>)
    {
        item_type value = buffer[0];
        return *reinterpret_cast<uint8_t*>(&value);
    }

    inline static int16_t __do_get(_Buffer& buffer, value_traits<int16_t>)
    {
        union {
            item_type buf[2];
            int16_t   value;
        } data;
        data.buf[0] = buffer[0];
        data.buf[1] = buffer[1];
        return data.value;
    }

    inline static uint16_t __do_get(_Buffer& buffer, value_traits<uint16_t>)
    {
        union {
            item_type buf[2];
            uint16_t  value;
        } data;
        data.buf[0] = buffer[0];
        data.buf[1] = buffer[1];
        return data.value;
    }
};
#endif

template<typename _Buffer>
struct buffer_read_helper
{
public:
    typedef typename buffer_traits<_Buffer>::item_type item_type;

    template<typename _Value>
    inline static std::size_t get_value(_Buffer& buffer, _Value* value, std::size_t count)
    {
        std::size_t length = count * sizeof(_Value) / sizeof(item_type);
        buffer.read(reinterpret_cast<item_type*>(value), length);
        return count;
    }

    template<typename _Value>
    inline static _Value get_value(_Buffer& buffer)
    {
#if 0
        return buffer_read_value<_Buffer, sizeof(item_type) == 1>::template get<_Value>(buffer);
#else
        _Value value;
        std::size_t length = sizeof(_Value) / sizeof(item_type);
        buffer.read(reinterpret_cast<item_type*>(&value), length);
        return value;
#endif
    }

    template<typename _Value>
    inline static std::size_t peek_value(_Buffer& buffer, _Value* value, std::size_t count)
    {
        std::size_t length = count * sizeof(_Value) / sizeof(item_type);
        buffer.peek(reinterpret_cast<item_type*>(value), length);
        return count;
    }

    template<typename _Value>
    inline static _Value peek_value(_Buffer& buffer)
    {
#if 0
        return buffer_read_value<_Buffer, sizeof(item_type) == 1>::template get<_Value>(buffer);
#else
        _Value value;
        std::size_t length = sizeof(_Value) / sizeof(item_type);
        buffer.peek(reinterpret_cast<item_type*>(&value), length);
        return value;
#endif
    }
};

template<typename _Buffer>
struct byteswap_buffer_read_helper
{
public:
    typedef typename buffer_traits<_Buffer>::item_type item_type;

    template<typename _Value, typename std::enable_if<(sizeof(_Value) == 1), int>::type = 0>
    inline static std::size_t get_value(_Buffer& buffer, _Value* value, std::size_t count)
    {
        std::size_t length = count * sizeof(_Value) / sizeof(item_type);
        buffer.read(reinterpret_cast<item_type*>(value), length);
        return count;
    }

    template<typename _Value, typename std::enable_if<(sizeof(_Value) > 1), int>::type = 0>
    inline static std::size_t get_value(_Buffer& buffer, _Value* value, std::size_t count)
    {
        std::size_t item_length = sizeof(_Value) / sizeof(item_type);
        std::size_t length = count * item_length;

        if(buffer.size() < length) {
            throw std::out_of_range("buffer::read()");
        }

        _Value temp;
        for(std::size_t idx = 0; idx < count; ++idx) {
            buffer.read(reinterpret_cast<item_type*>(&temp), item_length);
            value[idx] = byteswap(temp);
        }
        return length;
    }

    template<typename _Value>
    inline static _Value get_value(_Buffer& buffer)
    {
        _Value value;
        std::size_t length = sizeof(_Value) / sizeof(item_type);
        buffer.read(reinterpret_cast<item_type*>(&value), length);
        return byteswap(value);
    }

    template<typename _Value, typename std::enable_if<(sizeof(_Value) == 1), int>::type = 0>
    inline static std::size_t peek_value(_Buffer& buffer, _Value* value, std::size_t count)
    {
        std::size_t length = count * sizeof(_Value) / sizeof(item_type);
        buffer.peek(reinterpret_cast<item_type*>(value), length);
        return count;
    }

    template<typename _Value, typename std::enable_if<(sizeof(_Value) > 1), int>::type = 0>
    inline static std::size_t peek_value(_Buffer& buffer, _Value* value, std::size_t count)
    {
        std::size_t item_length = sizeof(_Value) / sizeof(item_type);
        std::size_t length = count * item_length;

        if(buffer.size() < length) {
            throw std::out_of_range("buffer::read()");
        }

        _Value temp;
        for(std::size_t idx = 0; idx < count; ++idx) {
            buffer.peek(reinterpret_cast<item_type*>(&temp), item_length);
            value[idx] = byteswap(temp);
        }
        return length;
    }

    template<typename _Value>
    inline static _Value peek_value(_Buffer& buffer)
    {
        _Value value;
        std::size_t length = sizeof(_Value) / sizeof(item_type);
        buffer.peek(reinterpret_cast<item_type*>(&value), length);
        return byteswap(value);
    }

    template<typename _Value>
    inline static _Value byteswap(_Value value)
    {
        return ss::utils::byteswap(value);
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
struct buffer_reader_traits
{
public:
    using item_type = typename buffer_traits<_Buffer>::item_type;
    using read_helper = typename std::conditional<_Network && (__BYTE_ORDER == __LITTLE_ENDIAN),
            byteswap_buffer_read_helper<_Buffer>,
            buffer_read_helper<_Buffer>>::type;

    template<typename _Value>
    inline static std::size_t get_value(_Buffer &buffer, _Value *value, std::size_t count)
    {
        return read_helper::get_value(buffer, value, count);
    }

    template<typename _Value>
    inline static _Value get_value(_Buffer &buffer)
    {
        return read_helper::template get_value<_Value>(buffer);
    }

    template<typename _Value>
    inline static std::size_t peek_value(_Buffer &buffer, _Value *value, std::size_t count)
    {
        return read_helper::peek_value(buffer, value, count);
    }

    template<typename _Value>
    inline static _Value peek_value(_Buffer &buffer)
    {
        return read_helper::template peek_value<_Value>(buffer);
    }

    inline static std::size_t tellg(_Buffer& buffer) noexcept
    {
        return buffer.tellg();
    }

    inline static void seekg(_Buffer& buffer, std::size_t offset) noexcept
    {
        buffer.seekg(offset);
    }

    inline static void sync(_Buffer& buffer) noexcept
    {
        buffer.sync();
    }

    inline static std::size_t size(_Buffer& buffer) noexcept
    {
        return buffer.size();
    }
};

} //detail

/**
 * virtual class of buffer reader
 */
class buffer_reader {
protected:
    template <typename _Tp>
    struct value_traits {
    };

public:
    virtual ~buffer_reader() noexcept = default;
//    virtual std::size_t read(uint8_t* data, std::size_t length) const = 0;

    virtual std::size_t tellg() const = 0;

    virtual void seekg(std::size_t offset) = 0;

    virtual std::size_t size() const noexcept = 0;

    virtual std::size_t available() const noexcept = 0;

    virtual uint8_t get_uint8() const = 0;
    virtual uint16_t get_uint16() const = 0;
    virtual uint32_t get_uint32() const = 0;
    virtual uint64_t get_uint64() const = 0;
    virtual float get_float() const = 0;
    virtual double get_double() const = 0;

    virtual std::size_t get_uint8(uint8_t* value, std::size_t length) const = 0;
    virtual std::size_t get_uint16(uint16_t* value, std::size_t length) const = 0;
    virtual std::size_t get_uint32(uint32_t* value, std::size_t length) const = 0;
    virtual std::size_t get_uint64(uint64_t* value, std::size_t length) const = 0;
    virtual std::size_t get_float(float* value, std::size_t length) const = 0;
    virtual std::size_t get_double(double* value, std::size_t length) const = 0;

    virtual uint8_t peek_uint8() const = 0;
    virtual uint16_t peek_uint16() const = 0;
    virtual uint32_t peek_uint32() const = 0;
    virtual uint64_t peek_uint64() const = 0;
    virtual float peek_float() const = 0;
    virtual double peek_double() const = 0;

    virtual std::size_t peek_uint8(uint8_t* value, std::size_t length) const = 0;
    virtual std::size_t peek_uint16(uint16_t* value, std::size_t length) const = 0;
    virtual std::size_t peek_uint32(uint32_t* value, std::size_t length) const = 0;
    virtual std::size_t peek_uint64(uint64_t* value, std::size_t length) const = 0;
    virtual std::size_t peek_float(float* value, std::size_t length) const = 0;
    virtual std::size_t peek_double(double* value, std::size_t length) const = 0;

    inline int8_t get_int8() const
    {
        auto ret = get_uint8();
        return *reinterpret_cast<int8_t*>(&ret);
    }

    inline int16_t get_int16() const
    {
        auto ret = get_uint16();
        return *reinterpret_cast<int16_t*>(&ret);
    }

    inline int32_t get_int32() const
    {
        auto ret = get_uint32();
        return *reinterpret_cast<int32_t*>(&ret);
    }

    inline int64_t get_int64() const
    {
        auto ret = get_uint64();
        return *reinterpret_cast<int64_t*>(&ret);
    }

    inline std::size_t get_int8(int8_t* value, std::size_t length) const
    {
        return this->get_uint8(reinterpret_cast<uint8_t*>(value), length);
    }

    inline std::size_t get_int16(int16_t* value, std::size_t length) const
    {
        return this->get_uint16(reinterpret_cast<uint16_t*>(value), length);
    }

    inline std::size_t get_int32(int32_t* value, std::size_t length) const
    {
        return this->get_uint32(reinterpret_cast<uint32_t*>(value), length);
    }

    inline std::size_t get_int64(int64_t* value, std::size_t length) const
    {
        return this->get_uint64(reinterpret_cast<uint64_t*>(value), length);
    }

    template<typename _Value>
    inline _Value get_value() const
    {
        return do_get_value(value_traits<_Value>());
    }
    template<typename _Value>
    inline std::size_t get_value(_Value* value, std::size_t length) const
    {
        return do_get_value(value, length);
    }

    inline int8_t peek_int8() const
    {
        auto ret = peek_uint8();
        return *reinterpret_cast<int8_t*>(&ret);
    }

    inline int16_t peek_int16() const
    {
        auto ret = peek_uint16();
        return *reinterpret_cast<int16_t*>(&ret);
    }

    inline int32_t peek_int32() const
    {
        auto ret = peek_uint32();
        return *reinterpret_cast<int32_t*>(&ret);
    }

    inline int64_t peek_int64() const
    {
        auto ret = peek_uint64();
        return *reinterpret_cast<int64_t*>(&ret);
    }

    inline std::size_t peek_int8(int8_t* value, std::size_t length) const
    {
        return this->peek_uint8(reinterpret_cast<uint8_t*>(value), length);
    }

    inline std::size_t peek_int16(int16_t* value, std::size_t length) const
    {
        return this->peek_uint16(reinterpret_cast<uint16_t*>(value), length);
    }

    inline std::size_t peek_int32(int32_t* value, std::size_t length) const
    {
        return this->peek_uint32(reinterpret_cast<uint32_t*>(value), length);
    }

    inline std::size_t peek_int64(int64_t* value, std::size_t length) const
    {
        return this->peek_uint64(reinterpret_cast<uint64_t*>(value), length);
    }

    template<typename _Value>
    inline _Value peek_value() const
    {
        return do_peek_value(value_traits<_Value>());
    }
    template<typename _Value>
    inline std::size_t peek_value(_Value* value, std::size_t length) const
    {
        return do_peek_value(value, length);
    }

private:
    inline uint8_t do_get_value(value_traits<int8_t>) const
    {
        return get_int8();
    }

    inline uint8_t do_get_value(value_traits<uint8_t>) const
    {
        return get_uint8();
    }

    inline int16_t do_get_value(value_traits<int16_t>) const
    {
        return get_int16();
    }

    inline uint16_t do_get_value(value_traits<uint16_t>) const
    {
        return get_uint16();
    }

    inline int32_t do_get_value(value_traits<int32_t>) const
    {
        return get_int32();
    }

    inline uint32_t do_get_value(value_traits<uint32_t>) const
    {
        return get_uint32();
    }

    inline int64_t do_get_value(value_traits<int64_t>) const
    {
        return get_int64();
    }

    inline uint64_t do_get_value(value_traits<uint64_t>) const
    {
        return get_uint64();
    }

    inline float do_get_value(value_traits<float>) const
    {
        return get_float();
    }

    inline double do_get_value(value_traits<double>) const
    {
        return get_double();
    }

    template <typename _Tp, typename std::enable_if<std::is_integral<_Tp>::value, int>::type = 0 >
    inline std::size_t do_get_value(_Tp* value, std::size_t length) const
    {
        return this->do_get_value(reinterpret_cast<typename std::make_unsigned<_Tp>::type *>(value), length);
    }

    inline std::size_t do_get_value(int8_t* value, std::size_t length) const
    {
        return get_int8(value, length);
    }

    inline std::size_t do_get_value(uint8_t* value, std::size_t length) const
    {
        return get_uint8(value, length);
    }

    inline std::size_t do_get_value(int16_t* value, std::size_t length) const
    {
        return get_int16(value, length);
    }

    inline std::size_t do_get_value(uint16_t* value, std::size_t length) const
    {
        return get_uint16(value, length);
    }

    inline std::size_t do_get_value(int32_t* value, std::size_t length) const
    {
        return get_int32(value, length);
    }

    inline std::size_t do_get_value(uint32_t* value, std::size_t length) const
    {
        return get_uint32(value, length);
    }

    inline std::size_t do_get_value(int64_t* value, std::size_t length) const
    {
        return get_int64(value, length);
    }

    inline std::size_t do_get_value(uint64_t* value, std::size_t length) const
    {
        return get_uint64(value, length);
    }

    inline std::size_t do_get_value(float* value, std::size_t length) const
    {
        return get_float(value, length);
    }

    inline std::size_t do_get_value(double* value, std::size_t length) const
    {
        return get_double(value, length);
    }

    inline uint8_t do_peek_value(value_traits<int8_t>) const
    {
        return peek_int8();
    }

    inline uint8_t do_peek_value(value_traits<uint8_t>) const
    {
        return peek_uint8();
    }

    inline int16_t do_peek_value(value_traits<int16_t>) const
    {
        return peek_int16();
    }

    inline uint16_t do_peek_value(value_traits<uint16_t>) const
    {
        return peek_uint16();
    }

    inline int32_t do_peek_value(value_traits<int32_t>) const
    {
        return peek_int32();
    }

    inline uint32_t do_peek_value(value_traits<uint32_t>) const
    {
        return peek_uint32();
    }

    inline int64_t do_peek_value(value_traits<int64_t>) const
    {
        return peek_int64();
    }

    inline uint64_t do_peek_value(value_traits<uint64_t>) const
    {
        return peek_uint64();
    }

    inline float do_peek_value(value_traits<float>) const
    {
        return peek_float();
    }

    inline double do_peek_value(value_traits<double>) const
    {
        return peek_double();
    }

    template <typename _Tp, typename std::enable_if<std::is_integral<_Tp>::value, int>::type = 0 >
    inline std::size_t do_peek_value(_Tp* value, std::size_t length) const
    {
        return this->do_peek_value(reinterpret_cast<typename std::make_unsigned<_Tp>::type*>(value), length);
    }

    inline std::size_t do_peek_value(int8_t* value, std::size_t length) const
    {
        return peek_int8(value, length);
    }

    inline std::size_t do_peek_value(uint8_t* value, std::size_t length) const
    {
        return peek_uint8(value, length);
    }

    inline std::size_t do_peek_value(int16_t* value, std::size_t length) const
    {
        return peek_int16(value, length);
    }

    inline std::size_t do_peek_value(uint16_t* value, std::size_t length) const
    {
        return peek_uint16(value, length);
    }

    inline std::size_t do_peek_value(int32_t* value, std::size_t length) const
    {
        return peek_int32(value, length);
    }

    inline std::size_t do_peek_value(uint32_t* value, std::size_t length) const
    {
        return peek_uint32(value, length);
    }

    inline std::size_t do_peek_value(int64_t* value, std::size_t length) const
    {
        return peek_int64(value, length);
    }

    inline std::size_t do_peek_value(uint64_t* value, std::size_t length) const
    {
        return peek_uint64(value, length);
    }

    inline std::size_t do_peek_value(float* value, std::size_t length) const
    {
        return peek_float(value, length);
    }

    inline std::size_t do_peek_value(double* value, std::size_t length) const
    {
        return peek_double(value, length);
    }
};

#if 0
template<typename _Buffer, typename _Traits>
class basic_buffer_reader {
public:
    using buffer_type = _Buffer;
    using traits_type = _Traits;

    explicit basic_buffer_reader(buffer_type& buffer) :
        _buffer(buffer)
    {
    }

    ~basic_buffer_reader()
    {
        this->sync();
    }

    template<typename _Value>
    inline _Value get_value() const
    {
        return traits_type::template get_value<_Value>(_buffer);
    }

    template<typename _Value>
    inline std::size_t get_value(_Value* value, std::size_t length) const
    {
        return traits_type::template get_value<_Value>(_buffer, value, length);
    }

    std::size_t tellg() const
    {
        return traits_type::tellg(_buffer);
    }

    void seekg(std::size_t offset)
    {
        return traits_type::seekg(_buffer, offset);
    }

    inline std::size_t size() const noexcept
    {
        return traits_type::size(_buffer);
    }

    inline std::size_t available() const noexcept
    {
        return traits_type::size(_buffer);
    }

    void sync() noexcept
    {
        return traits_type::sync(_buffer);
    }

private:
    buffer_type& _buffer;
};

#else
template<typename _Buffer, typename _Traits>
class basic_buffer_reader : public buffer_reader {
public:
    using buffer_type = _Buffer;
    using traits_type = _Traits;

    explicit basic_buffer_reader(buffer_type& buffer) :
        _buffer(buffer),
        _sync(true)
    {
    }

    basic_buffer_reader(buffer_type& buffer, bool sync) :
        _buffer(buffer),
        _sync(sync)
    {
    }

    ~basic_buffer_reader() noexcept
    {
        if (_sync) {
            this->sync();
        }
    }

    std::size_t tellg() const override
    {
        return traits_type::tellg(_buffer);
    }

    void seekg(std::size_t offset) override
    {
        return traits_type::seekg(_buffer, offset);
    }

    inline std::size_t size() const noexcept override
    {
        return traits_type::size(_buffer);
    }

    inline std::size_t available() const noexcept override
    {
        return traits_type::size(_buffer);
    }

    void sync() noexcept
    {
        return traits_type::sync(_buffer);
    }

//    inline size_t read(uint8_t* data, std::size_t length) const override
//    {
//        return _buffer.read(data, length);
//    }

    inline uint8_t get_uint8() const override
    {
        return traits_type::template get_value<uint8_t>(_buffer);
    }

    inline uint16_t get_uint16() const override
    {
        return traits_type::template get_value<uint16_t>(_buffer);
    }

    inline uint32_t get_uint32() const override
    {
        return traits_type::template get_value<uint32_t>(_buffer);
    }

    uint64_t get_uint64() const override
    {
        return traits_type::template get_value<uint64_t>(_buffer);
    }

    float get_float() const override
    {
        return traits_type::template get_value<float>(_buffer);
    }

    double get_double() const override
    {
        return traits_type::template get_value<double>(_buffer);
    }

    std::size_t get_uint8(uint8_t* value, std::size_t length) const override
    {
        return traits_type::get_value(_buffer, value, length);
    }

    std::size_t get_uint16(uint16_t* value, std::size_t length) const override
    {
        return traits_type::get_value(_buffer, value, length);
    }

    std::size_t get_uint32(uint32_t* value, std::size_t length) const override
    {
        return traits_type::get_value(_buffer, value, length);
    }

    std::size_t get_uint64(uint64_t* value, std::size_t length) const override
    {
        return traits_type::get_value(_buffer, value, length);
    }

    std::size_t get_float(float* value, std::size_t length) const override
    {
        return traits_type::get_value(_buffer, value, length);
    }

    std::size_t get_double(double* value, std::size_t length) const override
    {
        return traits_type::get_value(_buffer, value, length);
    }

    inline uint8_t peek_uint8() const override
    {
        return traits_type::template peek_value<uint8_t>(_buffer);
    }

    inline uint16_t peek_uint16() const override
    {
        return traits_type::template peek_value<uint16_t>(_buffer);
    }

    inline uint32_t peek_uint32() const override
    {
        return traits_type::template peek_value<uint32_t>(_buffer);
    }

    uint64_t peek_uint64() const override
    {
        return traits_type::template peek_value<uint64_t>(_buffer);
    }

    float peek_float() const override
    {
        return traits_type::template peek_value<float>(_buffer);
    }

    double peek_double() const override
    {
        return traits_type::template peek_value<double>(_buffer);
    }

    std::size_t peek_uint8(uint8_t* value, std::size_t length) const override
    {
        return traits_type::peek_value(_buffer, value, length);
    }

    std::size_t peek_uint16(uint16_t* value, std::size_t length) const override
    {
        return traits_type::peek_value(_buffer, value, length);
    }

    std::size_t peek_uint32(uint32_t* value, std::size_t length) const override
    {
        return traits_type::peek_value(_buffer, value, length);
    }

    std::size_t peek_uint64(uint64_t* value, std::size_t length) const override
    {
        return traits_type::peek_value(_buffer, value, length);
    }

    std::size_t peek_float(float* value, std::size_t length) const override
    {
        return traits_type::peek_value(_buffer, value, length);
    }

    std::size_t peek_double(double* value, std::size_t length) const override
    {
        return traits_type::peek_value(_buffer, value, length);
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
    buffer_type& _buffer;
    bool         _sync;
};
#endif

template<typename _Buffer>
using native_buffer_reader = basic_buffer_reader<_Buffer, detail::buffer_reader_traits<_Buffer, false>>;

template<typename _Buffer>
using network_buffer_reader = basic_buffer_reader<_Buffer, detail::buffer_reader_traits<_Buffer, true>>;

}
}

#endif //__STAR_SDK_UTILS_BUFFER_READER_H
