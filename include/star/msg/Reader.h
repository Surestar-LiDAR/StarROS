/**
 * @author   lucb
 * @date     2019/12/13
 */

#ifndef __STAR_SDK_MSG_READER_H
#define __STAR_SDK_MSG_READER_H

#include <star/Star.h>

#include <star/msg/StreamDecoder.h>
#include <star/msg/Dispatcher.h>

#include <star/Packet.h>
#include <star/Configure.h>
#include <star/Shot.h>

// #include <star/msg/scd/SCDSlowData.h

namespace ss {
namespace msg {

/**
 * 读取数据，该对象不是线程安全的。
 */
class __star_export Reader {
public:
    typedef enum {
        READER_INITIALIZE,
        READER_IDLE,
        READER_BUSY,
        READER_CLOSING,
        READER_CLOSED,
    } Status;
	
    Reader();
    virtual ~Reader();

    Reader(const Reader&) = delete;
    Reader(Reader&&) = delete;
    Reader& operator=(const Reader&) = delete;
    Reader& operator=(Reader&&) = delete;

    virtual void setup(const Configure& configure);
    virtual void open_debugs(const std::string& path, const cfg::DebugOutput& parameters);
    virtual void update_configure(Configure& configure);

    template <typename _Signature, typename _Handler>
    Reader& registerMessageHandler(uint64_t dataId, const _Handler& handler)
    {
        _dispatcher.registerMessageHandler<_Signature>(dataId, handler);
        return *this;
    }

    const Message* exec_once();

    std::size_t exec();

    virtual void reset();

    void close();

//    void exit();

    void set_stream_decoder(StreamDecoder* stream_decoder);

    /**
     * 设置一个新的流继续解析，并不会清空缓存
     * @param stream
     */
    bool open_stream(Stream* stream);

    void close_stream();

    /**
     * 设置为阻塞模式，阻塞模式中，exec函数在退出前不会返回
     */
    void set_block(bool block_mode);
    bool is_block() const;

    void set_water_mark(std::size_t water_mark);
    std::size_t water_mark() const;

    virtual bool wait_metadata();
    virtual void set_metadata(const ss::Configure& configure);

    bool start_record(Stream* stream);
    void stop_record();

    int64_t timestamp() const;
    void set_timestamp(int64_t timestamp);
#if 0
    Imp_MetaData_S& metadata();
    const Imp_MetaData_S& metadata() const;

    void set_metadata(const Imp_MetaData_S& metadata);
#endif

    Packet get_packet();

    bool eof() const;

    bool seek(int64_t pos);

    int64_t tell() const;

    int64_t size() const;

protected:
    const StreamDecoder* stream_decoder() const;
    uint64_t get_packet_count() const;

    virtual void reset_packet_count();

    virtual void save_point(const LasShot_S& lasShot);
    virtual void save_fast_synchron(const PosPPS_S& pps);
    virtual void save_slow_synchron(const PosPPS_S& pps);

    virtual void try_to_exit();
//    void push_gps_data(const scd::GPS& gps);
//    void push_imu_data(const scd::IMU& imu);

    virtual Packet unsafe_get_packets();
    void reserve_packets();

    Status status() const;
    bool exited() const;

    /**
     * 尝试向队列里插入数据
     * @tparam _Queue
     * @tparam _DataType
     * @param queue
     * @param data
     * @return
     */
    template <typename _Queue, typename _DataType>
    inline bool tryPushQueue(_Queue& queue, const _DataType& data);

protected:
    uint16_t                _checkLaserId;
    double                  _last_angle;

private:
    bool                    _block_mode;
    bool                    _exited;
    int64_t                 _will_seek;

#if 0
    mutable std::mutex      _stream_mutex;
    mutable std::mutex      _packet_mutex;
    std::condition_variable _closing_waiter;
#endif
    std::size_t             _water_mark;
    StreamDecoder*          _stream_decoder;
    Status                  _status;
    Dispatcher              _dispatcher;
    Packet                  _packets;
    Packet                  _cached_packets;
    int64_t                 _timestamp;
};

template<typename _Queue, typename _DataType>
bool Reader::tryPushQueue(_Queue& queue, const _DataType& data)
{
    if(_status == READER_INITIALIZE) {
        return false;
    }
    try {
#if 0
        std::lock_guard<decltype(this->_packet_mutex)> lock(this->_packet_mutex);
#endif
        queue.push_back(data);
        if (queue.size() > this->_water_mark) {
            return true;
        }
    }
    catch (const std::exception& ex) {
        (void)ex;
        TRACE("%s\n", ex.what());
        return true;
    }
    return false;
}

}
}

#endif //__STAR_SDK_MSG_READER_H
