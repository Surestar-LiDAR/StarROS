/**
 * @author xiaoma 
 * @date 2019/11/22
 */

#ifndef __STAR_MSG_CMD_PACKET_H
#define __STAR_MSG_CMD_PACKET_H

#include <star/Star.h>
#include <star/utils/singleton.h>
#include <star/utils/buffer_writer.h>
#include <star/utils/buffer_reader.h>

#include <cstddef>
#include <cstdint>
#include <memory>

namespace ss {
namespace msg {
namespace cmd {

/**
 * 前后缀辅助类
 * @tparam _Type 类型，为整形变量类型(如uint32_t)，表示前后缀的长度
 * @tparam value 值，前后缀的值(如0xa0a1a2a3)
 */
template<typename _Type, _Type value>
struct PacketFixable {
public:
    template<typename _Writer>
    ssize_t serialize(_Writer& __writer) noexcept;

    template<typename _Reader>
    ssize_t deserialize(_Reader& __reader) noexcept;

    inline static std::size_t size()
    {
        return sizeof(_Type);
    }
};

template<typename _Type, _Type value>
using Prefix = PacketFixable<_Type, value>;

template<typename _Type, _Type value>
using Suffix = PacketFixable<_Type, value>;

template<typename _Tp, std::size_t _ByteWidth>
struct PacketLength {
public:
    using data_type = _Tp;

    void setBodyLength(std::size_t length) noexcept
    {
        _length = length / _ByteWidth;
    }

    std::size_t bodyLength() const noexcept
    {
        return _length * _ByteWidth;
    }

    template<typename _Writer>
    ssize_t serialize(_Writer& __writer) noexcept
    {
        if (__writer.size() < this->size()) {
            return -1;
        }
        __writer.template put_value<data_type>(_length);
        return size();
    }

    template<typename _Reader>
    ssize_t deserialize(_Reader& __reader, std::size_t __length) noexcept
    {
        if (__length < size()) {
            return -1;
        }
        _length = __reader.template get_value<data_type>();
        return size();
    }

    inline static std::size_t size() noexcept
    {
        return sizeof(_length);
    }

private:
    data_type _length;
};

template<typename _Tp, typename _Calculator, std::size_t _Offset, std::size_t _Exclude>
struct PacketChecksum {
public:
    using data_type = _Tp;
    using calculator_type = _Calculator;

    template<typename _Writer>
    ssize_t serialize(_Writer& __writer) noexcept
    {
        if (__writer.size() < this->size()) {
            return -1;
        }
        auto& buffer = __writer.buf();
        _checksum = _calculator.calculate(buffer, _Offset, __writer.tellp() - _Offset - _Exclude);

        __writer.template put_value<data_type>(_checksum);
        return size();
    }

    template<typename _Reader>
    ssize_t deserialize(_Reader& __reader, std::size_t __length) noexcept
    {
        if (__length < size()) {
            return -1;
        }
        auto& buffer = __reader.buf();
        auto tellg = __reader.tellg();
        auto chksum = _calculator.calculate(buffer, _Offset, __reader.tellg() - _Offset - _Exclude);

        _checksum = __reader.template get_value<data_type>();

        if (chksum != _checksum) {
            return -1;
        }

        return size();
    }

    data_type checksum() const noexcept
    {
        return _checksum;
    }

    inline static std::size_t size() noexcept
    {
        return sizeof(_checksum);
    }


protected:
    data_type _checksum;
    calculator_type _calculator;
};

namespace detail {

struct packet_empty_item {
    template<typename _Writer>
    inline ssize_t serialize(_Writer& __writer) noexcept
    {
        (void) __writer;
        return 0;
    }

    template<typename _Reader>
    inline ssize_t deserialize(_Reader& __reader, std::size_t __length) noexcept
    {
        (void) __reader;
        (void) __length;
        return 0;
    }

    inline static std::size_t size() noexcept
    {
        return 0;
    }
};

struct packet_empty_header : public packet_empty_item {
public:
    inline void setBodyLength(std::size_t length) noexcept
    {
        (void) length;
    }

    inline std::size_t bodyLength() const noexcept
    {
        return 0;
    }
};

template<typename _Factory>
class __star_export packet_body_builder : public ss::utils::singleton<packet_body_builder<_Factory>> {
public:
    using body_type = typename _Factory::body_type;
    using key_type  = typename _Factory::key_type;

    using singleton = ss::utils::singleton<packet_body_builder<_Factory>>;

    packet_body_builder() = default;

    template<typename _Head>
    static body_type make(const _Head& head)
    {
        return singleton::getInstance()->_factory.make(head);
    }

    template<typename _Head>
    static std::size_t get_max_length(const _Head& head)
    {
        return singleton::getInstance()->_factory.max_length(head);
    }

private:
    _Factory _factory;
};

#define DECLARE_BODY_FACTORY(Packet) \
    SINGLETON_STATIC_DECLARE(Packet::body_builder)

template<typename _Tp, typename... _Args>
struct __has_static_make {
private :
    template<typename U>
    static auto Check(int) -> decltype(U::make(std::declval<_Args>()...), std::true_type());

    template<typename U>
    static auto Check(...) -> std::false_type;

public:
    static const bool value = decltype(Check<_Tp>(0))::value;
};

}

class Payload {
public:
	virtual ~Payload() = default;
	virtual ssize_t serialize(utils::buffer_writer& __writer) noexcept {return 0;}
    virtual ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept {return 0;}
};

template<
        typename _Prefix,
        typename _Head,
        typename _BodyFactory,
        typename _Tail,
        typename _Suffix>
class Packet {
public:
    using prefix_type  = typename std::conditional<std::is_void<_Prefix>::value,
            detail::packet_empty_item,
            _Prefix>::type;
    using body_factory = _BodyFactory;

    using head_type = typename std::conditional<std::is_void<_Head>::value,
            detail::packet_empty_header,
            typename std::conditional<std::is_integral<_Head>::value,
                    PacketLength<_Head, 1>,
                    _Head>::type >::type;

    using body_builder = typename std::conditional<
            detail::__has_static_make<body_factory, head_type>::value,
            _BodyFactory,
            detail::packet_body_builder<_BodyFactory> >::type;
    using body_type    = typename body_builder::body_type;

    using tail_type = typename std::conditional<std::is_void<_Tail>::value,
            detail::packet_empty_item,
            typename std::conditional<
                    std::is_integral<_Tail>::value,
                    PacketLength<_Tail, 1>,
                    _Tail>::type >::type;

    using suffix_type  = typename std::conditional<std::is_void<_Suffix>::value,
            detail::packet_empty_item,
            _Suffix>::type;

    Packet() noexcept;

    Packet(const Packet& other) noexcept;

    Packet(Packet&& other) noexcept;

    virtual uint64_t dataId() const noexcept = 0;
    virtual uint64_t identifier() const noexcept = 0;

    virtual ~Packet() noexcept;

    Packet& operator=(const Packet& other) noexcept;

    Packet& operator=(Packet&& other) noexcept;

    template<typename _Buffer>
    ssize_t serialize(_Buffer& buffer) noexcept;

    template<typename _Buffer>
    ssize_t deserialize(_Buffer& buffer) noexcept;

    template<typename _Buffer, typename _Traits>
    ssize_t serialize(utils::basic_buffer_writer<_Buffer, _Traits>& writer) noexcept;

    template<typename _Buffer, typename _Traits>
    ssize_t deserialize(utils::basic_buffer_reader<_Buffer, _Traits> & reader) noexcept;

    inline const body_type& body() const noexcept
    {
        return _body;
    }

    inline body_type& body() noexcept
    {
        return _body;
    }

    inline const head_type& head() const noexcept
    {
        return _head;
    }

    inline head_type& head() noexcept
    {
        return _head;
    }

    inline const tail_type& tail() const noexcept
    {
        return _tail;
    }

    inline tail_type& tail() noexcept
    {
        return _tail;
    }

    template <typename _Body>
    inline void setBody(const _Body& body)
    {
        _body = std::make_shared<_Body>(body);
    }

    template <typename _Body>
    inline void setBody(const std::shared_ptr<_Body>& body)
    {
        _body = body;
    }
	
    inline void setBody(const body_type& body) noexcept
    {
        _body = body;
    }

    inline void setHead(const head_type& head) noexcept
    {
        _head = head;
    }

    inline void setTail(const tail_type& tail) noexcept
    {
        _tail = tail;
    }

    inline std::size_t minsize() const noexcept
    {
        return _prefix.size() + _suffix.size() + _head.size() + _tail.size();
    }

private:
    prefix_type _prefix;
    head_type _head;
    body_type _body;
    tail_type _tail;
    suffix_type _suffix;
};

}
}
}

#endif //__STAR_MSG_CMD_PACKET_H

#include <star/msg/cmd/Packet.tcc>
