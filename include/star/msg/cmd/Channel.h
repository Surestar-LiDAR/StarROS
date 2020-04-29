/**
 * @author   lucb
 * @date     2020/2/27
 */

#ifndef __STAR_SDK_MSG_CHANNEL_H
#define __STAR_SDK_MSG_CHANNEL_H

#include <star/Stream.h>

#include <star/utils/bytes_order.h>
#include <star/utils/circular_buffer.h>
#include <star/utils/buffer_writer.h>
#include <star/utils/buffer_reader.h>

#include <star/utils/buffer.h>
#include <star/utils/mutex.h>
#include <star/utils/thread.h>
#include <star/utils/condition_variable.h>

#include <system_error>

namespace ss {
namespace msg {
namespace cmd {

std::error_code __star_export system_error_code();

namespace detail {

    template <bool swap, typename _Buffer>
    struct make_buffer_operator
    {
        using reader = utils::native_buffer_reader<_Buffer>;
        using writer = utils::native_buffer_writer<_Buffer>;
    };

    template <typename _Buffer>
    struct make_buffer_operator<true, _Buffer>
    {
        using reader = utils::network_buffer_reader<_Buffer>;
        using writer = utils::network_buffer_writer<_Buffer>;
    };

    template <typename _Channel, typename _Packet>
    struct OnChannelReceiveCallbackInvoker {
        virtual ~OnChannelReceiveCallbackInvoker() = default;
        virtual void invoke(const _Channel* channel, _Packet&& packet) = 0;
    };

    template <typename _Channel>
    struct OnChannelExceptCallbackInvoker {
        virtual ~OnChannelExceptCallbackInvoker() = default;
        virtual void invoke(const _Channel* channel, const std::exception& ex) = 0;
    };

    template <typename _Channel, typename _Packet, typename _Callback>
    struct OnChannelReceiveCallbackInvokerImpl : public OnChannelReceiveCallbackInvoker<_Channel, _Packet> {
    public:
        void invoke(const _Channel* channel, _Packet&& packet) override
        {
            _callback(channel, std::move(packet));
        }

    private:
        _Callback _callback;
    };

    template <typename _Channel, typename _Callback>
    struct OnChannelExceptCallbackInvokerImpl : public OnChannelExceptCallbackInvoker<_Channel> {
    public:
        void invoke(const _Channel* channel, const std::exception& ex) override
        {
            _callback(channel, ex);
        }
    private:
        _Callback _callback;
    };
}

template <typename _Packet, int _ByteOrder = __LITTLE_ENDIAN>
class Channel {
public:
    using packet_type = _Packet;
    using read_buffer = utils::circular_buffer<char>;
    using write_buffer = utils::mutable_buffer<char>;
    using buffer_reader = typename detail::make_buffer_operator<__BYTE_ORDER != _ByteOrder, read_buffer>::reader;
    using buffer_writer = typename detail::make_buffer_operator<__BYTE_ORDER != _ByteOrder, write_buffer>::writer;

    constexpr static std::size_t __swap_size = 4096;

    Channel() :
        _stream(nullptr),
        _buffer(8192),
        _closed(false),
        _read_swap{},
        _write_swap{}
    {
    }

    explicit Channel(std::size_t capacity) :
        _stream(nullptr),
        _buffer(capacity),
        _closed(false),
        _read_swap{},
        _write_swap{}
    {
    }

    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;

    Channel(Channel&& other) noexcept :
        _stream(other._stream),
        _buffer(std::move(other._buffer)),
        _closed(other._closed),
        _read_swap{},
        _write_swap{}
//        _read_mutex(std::move(other._read_mutex)),
//        _write_mutex(std::move(other._write_mutex)),
//        _stream_mutex(std::move(other._stream_mutex))
	
    {
        other._stream = nullptr;
		other._closed = true;	    
    }
	
    Channel& operator=(Channel&& other) noexcept
    {
        this->doClose();
        std::swap(_stream, other._stream);
        std::swap(_buffer, other._buffer);
        std::swap(_closed, other._closed);
//        std::swap(_read_mutex, other._read_mutex);
//        std::swap(_write_mutex, other._write_mutex);
//        std::swap(_stream_mutex, other._stream_mutex);
    	return *this;
    }

	virtual ~Channel()
    {
        this->doClose();
        this->doDestroy();
    }

    bool open(Stream* stream)
    {
        std::lock_guard<decltype(_read_mutex)> locker(_read_mutex);
        std::lock_guard<decltype(_write_mutex)> locker1(_write_mutex);
        if(_stream != nullptr) {
            return false;
        }
        _stream = stream;
        return true;
    }

    virtual void close()
    {
        this->doClose();
        this->doDestroy();
    }

    packet_type get()
    {
        return this->read();
    }

    packet_type read()
    {
        packet_type packet;
        
        while (!this->closed()) {
            while (_buffer.size() < packet.minsize()) {
                underflow();
            }
            
            buffer_reader reader(_buffer);
            if (packet.deserialize(reader) > 0) {
                return packet;
            }
        	
            underflow();
            
            // for (int idx = 0; idx < len; ++idx) {
            //     printf("%02X ", _buffer[idx] & 0x00ffu);
            // }
            // printf("\n");
        }
        throw std::system_error(std::make_error_code(std::errc::bad_file_descriptor));
    }

    void write(packet_type& packet)
    {
        if(this->closed()) {
            throw std::system_error(std::make_error_code(std::errc::bad_file_descriptor));
        }

        write_buffer writeBuffer(_write_swap, __swap_size);
        buffer_writer writer(writeBuffer);
        const ssize_t len = packet.serialize(writer);
        if(len < 0) {
            throw std::system_error(std::make_error_code(std::errc::invalid_argument));
        }

        // for(int idx = 0; idx < len; ++idx) {
        //     printf("%02X ", _write_swap[idx] & 0x00ffu);
        // }
        // printf("\n");

        std::lock_guard<decltype(_write_mutex)> write_locker(_write_mutex);
        const ssize_t ret = _stream->write(_write_swap, len);
        if(ret < 0) {
            throw std::system_error(system_error_code());
        }
    }

	bool closed() const
    {
        return _stream == nullptr || this->_closed || !(this->_stream->good());
    }

protected:
	void underflow()
	{
        std::lock_guard<decltype(_read_mutex)> write_locker(_read_mutex);
        const ssize_t len = _stream->read(_read_swap, __swap_size);

        if (len < 0) {
            throw std::system_error(system_error_code());
        }
        _buffer.write(_read_swap, len);
	}

    void on_error()
    {
        this->_closed = true;
    }
	
    void doClose()
    {
        _closed = true;

        if (_stream != nullptr) {
            std::lock_guard<decltype(_read_mutex)> locker(_read_mutex);
            std::lock_guard<decltype(_write_mutex)> locker1(_write_mutex);
            if (_stream != nullptr) {
                _stream->close();
            }
        }

    }
	void doDestroy()
    {
        std::lock_guard<decltype(_read_mutex)> locker(_read_mutex);
        std::lock_guard<decltype(_write_mutex)> locker1(_write_mutex);
        delete _stream;
        _stream = nullptr;
    }

private:
    Stream*         _stream;
    read_buffer     _buffer;
    bool            _closed;
    char            _read_swap[__swap_size];
    char            _write_swap[__swap_size];
    std::mutex      _read_mutex;
    std::mutex      _write_mutex;
};

template <typename _Packet>
class Response {
public:
    using packet_type = _Packet;
    Response() :
        _waiting(false),
        _id(0)
    {
    }

    ~Response()
    {
        if (this->waiting()) {
            this->_waiter.notify_all();
        }
    }

    void close()
    {
        this->_ec = std::make_error_code(std::errc::network_down);
        this->_waiter.notify_all();
    }

    bool response(const packet_type& packet)
    {
        std::unique_lock<std::mutex> locker(_mutex);
        if (!this->waiting()) {
            return false;
        }
        if (_waiting && this->_id == packet.identifier()) {
            _packet = packet;
            _waiter.notify_one();
            this->_waiting = false;
            return true;
        }
        return false;
    }

    bool response(packet_type&& packet)
    {
        std::unique_lock<std::mutex> locker(_mutex);
        if (!this->waiting()) {
            return false;
        }
        if (this->_waiting && this->_id == packet.identifier()) {
            _packet = std::move(packet);
            _waiter.notify_one();
            this->_waiting = false;
            return true;
        }
        return false;
    }

    void set_error_code(const std::error_code& ec)
    {
        std::unique_lock<std::mutex> locker(_mutex);
        if (this->waiting()) {
            _ec = ec;
            _waiter.notify_one();
        }
    }

    void start(uint64_t id)
    {
        std::unique_lock<std::mutex> locker(_mutex);
        this->_waiting = true;
        this->_id = id;
    }

    packet_type wait(int64_t timeout)
    {
        std::error_code ec;
        packet_type packet = wait(timeout, ec);
        if (ec) {
            throw std::system_error(ec);
        }
        return packet;
    }

    packet_type wait(int64_t timeout, std::error_code& ec)
    {
        std::unique_lock<std::mutex> locker(_mutex);
        if (!this->waiting()) {
            _ec = {};
            return _packet;
        }
        const auto status = _waiter.wait_for(locker, std::chrono::milliseconds(timeout));
        if (status != std::cv_status::no_timeout) {
            ec = std::make_error_code(std::errc::timed_out);
        }
        else if (_ec) {
            ec = _ec;
        }
        else {
            ec = system_error_code();
        }
        return _packet;
    }

    bool waiting() const
    {
        return _waiting;
    }

private:
    bool                    _waiting;
    uint64_t                _id;
    packet_type             _packet;
    std::mutex              _mutex;
    std::condition_variable _waiter;
    std::error_code         _ec;
};
	
template <typename _Packet, int _ByteOrder = __LITTLE_ENDIAN>
class AsyncChannel : public Channel<_Packet, _ByteOrder> {
public:
    using parent_type = Channel<_Packet, _ByteOrder>;
    using packet_type = typename parent_type::packet_type;
    using read_buffer = typename parent_type::read_buffer;
    using write_buffer = typename parent_type::write_buffer;
    using buffer_reader = typename parent_type::buffer_reader;
    using buffer_writer = typename parent_type::buffer_writer;
    using receive_callback_invoker = detail::OnChannelReceiveCallbackInvoker<AsyncChannel, packet_type>;
    using error_callback_invoker = detail::OnChannelExceptCallbackInvoker<AsyncChannel>;

    AsyncChannel() :
        parent_type(),
        _receive_callback(nullptr),
        _except_callback(nullptr)
    {
    }

    explicit AsyncChannel(std::size_t capacity) :
        parent_type(capacity),
        _receive_callback(nullptr),
        _except_callback(nullptr)
    {
    }

    AsyncChannel(const AsyncChannel&) = delete;
    AsyncChannel& operator=(const AsyncChannel&) = delete;

    AsyncChannel(AsyncChannel&& other) noexcept :
        parent_type(std::move(other)),
        _read_thread(std::move(other._read_thread)),
		_response(std::move(other._response)),
		_receive_callback(other._receive_callback),
		_except_callback(other._except_callback)
    {
        other._receive_callback = nullptr;
        other._except_callback = nullptr;
    }

    AsyncChannel& operator=(AsyncChannel&& other) noexcept
    {
        this->do_close();
        delete _receive_callback;
        _receive_callback = nullptr;
        delete _except_callback;
        _except_callback = nullptr;
    	
        parent_type::operator=(std::move(other));
        std::swap(_read_thread, other._read_thread);
        std::swap(_response, other._response);
        std::swap(_receive_callback, other._receive_callback);
        std::swap(_except_callback, other._except_callback);
        return *this;
    }

	~AsyncChannel()
    {
        do_close();
        delete _receive_callback;
        delete _except_callback;
    }

    bool start()
    {
        if(_read_thread.joinable()) {
            return false;
        }
        _read_thread = std::thread([this]() {
            this->read_thread();
        });
        return true;
    }

    //todo 确保stream对象的安全删除
    void close() override
    {
        do_close();
    }

    void read_thread()
    {
        try {
            while(!this->closed()) {
                packet_type packet = this->read();
                if(_response.response(packet)) {
                    continue;
                }

                std::lock_guard<decltype(this->_mutex)> locker(this->_mutex);
                if(_receive_callback != nullptr) {
                    _receive_callback->invoke(this, std::move(packet));
                }
            }
        }
        catch (const std::exception& ex) {
            this->on_error();
            std::lock_guard<decltype(this->_mutex)> locker(this->_mutex);
            if(_except_callback != nullptr) {
                _except_callback->invoke(this, ex);
            }
            _response.set_error_code(system_error_code());
        }
    }

    packet_type wait_response(uint64_t id, int64_t timeout = 1000)
    {
        return _response.get(id, timeout);
    }

    packet_type wait_response(uint64_t id, int64_t timeout, std::error_code& ec)
    {
        return _response.get(id, timeout, ec);
    }

    packet_type wait_response(uint64_t id, std::error_code& ec)
    {
        return _response.get(id, 1000, ec);
    }

    packet_type request(packet_type& command, int64_t timeout)
    {
        _response.start(command.identifier());
        this->write(command);
        return _response.wait(timeout);
    }

    packet_type request(packet_type& command, int64_t timeout, std::error_code& ec)
    {
        try {
            _response.start(command.identifier());
            this->write(command);
            return _response.wait(timeout, ec);
        }
        catch(const std::exception& ) {
            ec = system_error_code();
        }
        return {};
    }

    packet_type request(packet_type& command, std::error_code& ec)
    {
        return this->request(command, 1000, ec);
    }

    template <typename _Callback>
    bool set_receive_callback(const _Callback& callback)
    {
        std::lock_guard<decltype(this->_mutex)> locker(this->_mutex);
        if(_receive_callback != nullptr) {
            return false;
        }

        _receive_callback = new detail::OnChannelReceiveCallbackInvokerImpl<AsyncChannel, packet_type, _Callback>(callback);
        return true;
    }

    template <typename _Callback>
    bool set_exception_callback(const _Callback& callback)
    {
        std::lock_guard<decltype(this->_mutex)> locker(this->_mutex);
        if(_receive_callback != nullptr) {
            return false;
        }
        _except_callback = new detail::OnChannelExceptCallbackInvokerImpl<AsyncChannel, _Callback>(callback);
        return true;
    }

protected:
	void do_close()
	{
        this->doClose();

        _read_thread.join();
        _response.close();
		
        this->doDestroy();
	}
	
private:
    std::thread                 _read_thread;
    Response<packet_type>       _response;
    std::mutex                  _mutex;
    receive_callback_invoker*   _receive_callback;
    error_callback_invoker*     _except_callback;
};

}
}
}

#endif //__STAR_SDK_MSG_CHANNEL_H
