/**
 * @author xiaoma 
 * @date 2019/11/22
 */
#ifndef __LIGHT_PACKET_TCC
#define __LIGHT_PACKET_TCC

#include <star/Star.h>
#include <star/msg/cmd/Packet.h>

namespace ss {
namespace msg {
namespace cmd {

template<typename _Type, _Type value>
template<typename _Writer>
ssize_t PacketFixable<_Type, value>::serialize(_Writer& __writer) noexcept
{
    __writer.put_value(value);
    return sizeof(_Type);
}

template<typename _Type, _Type value>
template<typename _Reader>
ssize_t PacketFixable<_Type, value>::deserialize(_Reader& __reader) noexcept
{
    _Type __value = __reader.template get_value<_Type>();
    if (value == __value) {
        return sizeof(_Type);
    }
    return -1;
}

namespace detail {
template<typename _Tp>
struct packet_body_traits {
    static void init(_Tp& __tp) noexcept
    {
        (void) __tp;
    }

    static void destroy(_Tp& __tp) noexcept
    {
        (void) __tp;
    }

    static _Tp copy(const _Tp& __tp) noexcept
    {
        return __tp;
    }

    static bool is_null(const _Tp& __tp) noexcept
    {
        (void) __tp;
        return false;
    }

    template<typename _Writer>
    static ssize_t serialize(_Tp& __tp, _Writer& __writer) noexcept
    {
        return __tp.serialize(__writer);
    }

    template<typename _Reader>
    static ssize_t deserialize(_Tp& __tp, _Reader& __reader, std::size_t __length) noexcept
    {
        return __tp.deserialize(__reader, __length);
    }
};

template<typename _Tp>
struct packet_body_traits<std::shared_ptr<_Tp>> {
    static void init(std::shared_ptr<_Tp>& __tp) noexcept
    {
        (void) __tp;
    }

    static void destroy(std::shared_ptr<_Tp>& __tp) noexcept
    {
        (void) __tp;
    }

    static std::shared_ptr<_Tp> copy(const std::shared_ptr<_Tp>& __tp) noexcept
    {
        return __tp;
    }

    static bool is_null(const std::shared_ptr<_Tp>& __tp) noexcept
    {
        return __tp.get() == nullptr;
    }

    template<typename _Writer>
    static ssize_t serialize(std::shared_ptr<_Tp>& __tp, _Writer& __writer) noexcept
    {
        return __tp->serialize(__writer);
    }

    template<typename _Reader>
    static ssize_t deserialize(std::shared_ptr<_Tp>& __tp, _Reader& __reader, std::size_t __length) noexcept
    {
        return __tp->deserialize(__reader, __length);
    }
};

template<typename _Tp>
struct packet_body_traits<_Tp*> {
    static void init(_Tp*& __tp) noexcept
    {
        __tp = nullptr;
    }

    static void destroy(_Tp*& __tp) noexcept
    {
        delete __tp;
    }

    static _Tp* copy(const _Tp*& __tp) noexcept
    {
        return __tp->clone();
    }

    static bool is_null(const _Tp* __tp) noexcept
    {
        return __tp == nullptr;
    }

    template<typename _Reader>
    static ssize_t serialize(_Tp* __tp, _Reader& __reader) noexcept
    {
        return __tp->serialize(__reader);
    }

    template<typename _Writer>
    static ssize_t deserialize(_Tp* __tp, _Writer& __writer, std::size_t __length) noexcept
    {
        return __tp->deserialize(__writer, __length);
    }
};

#if 0
struct packet_body_traits {
    void init(_Tp& __tp) noexcept
    {
        using __traits = packet_body_traits<_Tp>;
        __traits::init(__tp);
    }

    template <typename _Tp>
    void destroy(_Tp& __tp) noexcept
    {
        using __traits = packet_body_traits<_Tp>;
        __traits::destroy(__tp);
    }

    template <typename _Tp>
    _Tp copy(_Tp& __tp) noexcept
    {
        using __traits = packet_body_traits<_Tp>;
        return __traits::copy(__tp);
    }

    template <typename _Tp>
    bool is_null(_Tp& __tp) noexcept
    {
        using __traits = packet_body_traits<_Tp>;
        return __traits::is_null(__tp);
    }

    template <typename _Tp, typename _Reader>
    ssize_t deserialize(_Tp& __tp, _Reader& __reader, std::size_t __length) noexcept
    {
        using __traits = packet_body_traits<_Tp>;
        return __traits::deserialize(__tp, __reader, __length);
    }

    template <typename _Tp, typename _Reader>
    ssize_t serialize(_Tp& __tp, _Reader& __reader) noexcept
    {
        using __traits = packet_body_traits<_Tp>;
        return __traits::serialize(__tp, __reader);
    }
};
#endif
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::Packet() noexcept :
        _prefix(),
        _head(),
        _tail(),
        _suffix()
{
    detail::packet_body_traits<body_type>::init(_body);
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::Packet(const Packet& other) noexcept :
        _prefix(other._prefix),
        _head(other._head),
        _tail(other._tail),
        _suffix(other._suffix)
{
    _body = detail::packet_body_traits<body_type>::copy(other._body);
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::Packet(Packet&& other) noexcept :
        _prefix(std::move(other._prefix)),
        _head(std::move(other._head)),
        _tail(std::move(other._tail)),
        _suffix(std::move(other._suffix))
{
    detail::packet_body_traits<body_type>::init(_body);
    std::swap(_body, other._body);
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::~Packet() noexcept
{
    detail::packet_body_traits<body_type>::destroy(_body);
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>&
Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::operator=(const Packet& other) noexcept
{
    _prefix = other._prefix;
    _head = other._head;
    _body = detail::packet_body_traits<body_type>::copy(other._body);
    _tail = other._tail;
    _suffix = other._suffix;
    return *this;
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>&
Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::operator=(Packet&& other) noexcept
{
    std::swap(_prefix, other._prefix);
    std::swap(_head, other._head);
    std::swap(_body, other._body);
    std::swap(_tail, other._tail);
    std::swap(_suffix, other._suffix);
    return *this;
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
template<typename _Buffer>
ssize_t Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::serialize(_Buffer& buffer) noexcept
{
    const std::size_t __min_size = minsize();

    if (buffer.available() < __min_size) {
        return -1;
    }

    utils::native_buffer_writer<_Buffer> __writer(buffer);

    return serialize(__writer);
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
template<typename _Buffer>
ssize_t Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::deserialize(_Buffer& buffer) noexcept
{
    const std::size_t __min_size = minsize();

    if (buffer.length() < __min_size) {
        return -1;
    }

    utils::native_buffer_reader <_Buffer> __reader(buffer, buffer.length());

    return deserialize(__reader);
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
template<typename _Buffer, typename _Traits>
ssize_t Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::serialize(utils::basic_buffer_writer<_Buffer, _Traits>& __writer) noexcept
{
    try {
        if (_prefix.serialize(__writer) < 0) {
            return -1;
        }
        const std::size_t __head_offset = _prefix.size();
        const std::size_t __body_offset = __head_offset + _head.size();
        std::size_t __body_length = 0;

        if (!detail::packet_body_traits<body_type>::is_null(_body)) {
            __writer.seekp(__body_offset);
            ssize_t body_length = detail::packet_body_traits<body_type>::serialize(_body, __writer);
            if (body_length < 0) {
                return -1;
            }
            __body_length = static_cast<std::size_t>(body_length);
        }

        __writer.seekp(__head_offset);

        _head.setBodyLength(__body_length);
        if (_head.serialize(__writer) < 0) {
            return -1;
        }

        const std::size_t __tail_offset = __body_offset + __body_length;

        __writer.seekp(__tail_offset);
        if (_tail.serialize(__writer) < 0) {
            return -1;
        }

        if (_suffix.serialize(__writer) < 0) {
            return -1;
        }

        return __writer.tellp();
    }
    catch (const std::exception&) {
        return -1;
    }
}

template<typename _Prefix, typename _Head, typename _BodyFactory, typename _Tail, typename _Suffix>
template<typename _Buffer, typename _Traits>
ssize_t Packet<_Prefix, _Head, _BodyFactory, _Tail, _Suffix>::deserialize(utils::basic_buffer_reader<_Buffer, _Traits>& __reader) noexcept
{
    const std::size_t __min_size = minsize();
    try {
        while (__reader.size() >= __min_size) {
            std::size_t __offset = __reader.tellg();

            if (_prefix.deserialize(__reader) < 0) {
                __reader.seekg(__offset + 1);
                continue;
            }

            if (_head.deserialize(__reader, __reader.size()) < 0) {
                __reader.seekg(__offset + 1);
                continue;
            }

            const std::size_t __body_length = _head.bodyLength();

            if (__reader.size() < __body_length) { //长度不够
                //todo 检查是否合法
                std::size_t __max_length = body_builder::get_max_length(_head);
                if (__max_length == 0) {
                    __reader.seekg(__offset + 1);
                } 
                else if (__body_length > __max_length) {
                    __reader.seekg(__offset + 1);
                }
                continue;
            }

            const std::size_t __body_offset = __reader.tellg();
            const std::size_t __tail_offset = __body_length + __body_offset;
            const std::size_t __suffix_offset = __tail_offset + _tail.size();

            __reader.seekg(__suffix_offset);
            if (_suffix.deserialize(__reader, __reader.size()) < 0) {
                __reader.seekg(__offset + 1);
                continue;
            }

            __reader.seekg(__tail_offset);
            if (_tail.deserialize(__reader, __reader.size()) < 0) {
                __reader.seekg(__offset + 1);
                continue;
            }

            _body = body_builder::make(_head);

            if (detail::packet_body_traits<body_type>::is_null(_body)) {
                __reader.seekg(__offset + 1);
                continue;
            }

            __reader.seekg(__body_offset);
            if (detail::packet_body_traits<body_type>::deserialize(_body, __reader, __body_length) < 0) {
                __reader.seekg(__offset + 1);
                continue;
            }

            __reader.seekg(__suffix_offset + _suffix.size() - __offset);
            return __suffix_offset + _suffix.size() - __offset;
        }
    }
    catch (const std::exception&) {
        return -1;
    }

    return -1;
}

}
}
}

#endif //__LIGHT_PACKET_TCC
