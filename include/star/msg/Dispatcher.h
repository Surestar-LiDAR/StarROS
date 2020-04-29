/**
 * @author   lucb
 * @date     2019/12/13
 */

#ifndef __STAR_SDK_MESSAGE_HANDLER_H
#define __STAR_SDK_MESSAGE_HANDLER_H

#include <star/Star.h>

#include <star/msg/Message.h>

#include <unordered_map>

namespace ss {
namespace msg {

struct MessageHandler {
public:
    virtual ~MessageHandler() = default;
    virtual bool exec(const Message* message) = 0;
};

namespace detail {

template <typename _Signature>
struct __message_handler_exec;

template <typename _Ret, typename _Message>
struct __message_handler_exec<_Ret(_Message*)>
{
    template <typename _Handler>
    static bool exec(_Handler& handler, const Message* message)
    {
        const _Message* msg = dynamic_cast<const _Message>(message);
        if(msg == nullptr) {
            return false;
        }
        handler(msg);
        return true;
    }
};

template <typename _Message>
struct __message_handler_exec<void(_Message*)>
{
    template <typename _Handler>
    static bool exec(_Handler& handler, const Message* message)
    {
        const _Message* msg = dynamic_cast<const _Message*>(message);
        if (msg == nullptr) {
            return false;
        }
        handler(msg);
        return true;
    }
};

template <typename _Signature, typename _Handler>
struct BasicMessageHandler : public MessageHandler {
    explicit BasicMessageHandler(const _Handler& handler) :
        _handler(handler)
    {
    }

    bool exec(const Message* message) override
    {
        return detail::__message_handler_exec<_Signature>::exec(_handler, message);
    }

private:
    _Handler _handler;
};

}

class __star_export Dispatcher {
public:
    using handler_map_t = std::unordered_multimap<uint64_t, MessageHandler*>;

    ~Dispatcher();

    template <typename _Signature, typename _Handler>
    Dispatcher& registerMessageHandler(uint64_t dataId, const _Handler& handler);

//    template <typename _Signature, typename... _Handler>
//    Dispatcher& registerMessageHandler(uint64_t dataId, const _Handler&... handler);

    bool dispatch(const Message* message);
//    bool dispatch(uint64_t dataId, const Message* message);

    bool empty() const;
private:
    handler_map_t _handlers;
};

template<typename _Signature, typename _Handler>
Dispatcher& Dispatcher::registerMessageHandler(uint64_t dataId, const _Handler& handler)
{
    MessageHandler* messageHandler = new detail::BasicMessageHandler<_Signature, _Handler>(handler);
    _handlers.insert(std::make_pair(dataId, messageHandler));
    return *this;
}

//template<typename _Signature, typename... _Handler>
//Dispatcher& Dispatcher::registerMessageHandler(uint64_t dataId, const _Handler& ... handler)
//{
//    registerMessageHandler(dataId, handler);
//    registerMessageHandler(dataId, ...);
//}

}
}

#endif //__STAR_SDK_MESSAGE_HANDLER_H
