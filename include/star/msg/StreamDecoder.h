/**
 * @author   lucb
 * @date     2019/12/11
 */

#ifndef __STAR_SDK_DECODER_H
#define __STAR_SDK_DECODER_H

#include <star/Star.h>

#include <star/Stream.h>
#include <star/msg/Message.h>

#include <star/utils/circular_buffer.h>
#include <star/utils/mutex.h>

#include <system_error>

#include <new>
#include <map>

namespace ss {
namespace msg {

namespace detail {

class MsgBuilder {
public:
    virtual ~MsgBuilder() = default;

    virtual msg::Message& get() = 0;
};

template<typename _Msg>
class MsgBuilderImpl : public MsgBuilder {
public:
    msg::Message& get() override
    {
        new(&_storage)_Msg();
        return _storage;
    }

private:
    _Msg _storage;
};

}

/**
 * StreamDecoder 解码数据流
 * @node 使用该对象时应留意@see decode()函数返回的@see Message对象的生命周期
 */
class __star_export StreamDecoder {
public:
    StreamDecoder();

    virtual ~StreamDecoder();

    void setByteOrder(bool useNetworkByteOrder);

    /**
     * 设置需要解析的数据流对象
     * Stream对象的生命周期由StreamDecoder管理
     * @param stream 新的stream对象
     */
    void setSourceStream(Stream* stream);

    void closeSourceStream();

    /**
     * 从数据流中解码一个数据包
     * @return 解析到的数据包@see Message，失败返回空
     * @note 返回的数据包对象为临时对象地址，指向的对象内存由StreamDecoder管理
     *          当下一次调用decode函数，或者StreamDecoder对象被释放时，
     *          返回的地址指向的内存资源被释放。
     */
    virtual const Message* read();

    //todo
    virtual ssize_t write(const Message* message);

    /**
     * 等同于@see read();
     */
    virtual const Message* decode()
    {
        return read();
    }

    /**
     * 注册一个数据包类型
     * @tparam _Msg 注册的数据包类型，必须是@see Message的派生类
     * @param dataId 数据包对应的dataId
     * @return 注册成功则返回true，失败返回false
     */
    template<typename _Msg>
    bool material(uint64_t dataId);

    /**
     * 判断数据流是否结束
     * @return 结束则返回true，否则返回false
     */
    bool eof() const;

    bool seek(int64_t pos);

    int64_t tell() const;

    int64_t size() const;

    /**
     * 关闭stream
     */
    void close();

    uint64_t totalByteCount() const;
    void resetTotalByteCount();

    double progress() const;

    virtual bool startRecord(Stream* stream);
    virtual void stopRecord();

protected:
    /**
     * 从数据流中解码一个数据包
     * @return 解析到的数据包@see Message，失败返回空
     * @note 返回的数据包对象为临时地址，指向的对象内存由StreamDecoder管理
     *          当下一次调用decode函数，或者StreamDecoder对象被释放时，
     *          返回的引用所指向的内存资源被释放。
     */
    virtual const Message* decodeMessage() = 0;

    /**
     * 解码获取一个数据包，该函数在一般接口函数@see doDecode()中调用
     * @param dataId 从数据流中解析出来的dataId
     * @return 解析出来的数据包@see Message，失败返回空
     * @note 返回的数据包对象为临时对象，生命周期到下一次调用decode函数结束。
     */
    const Message* decodeWithDataId(uint64_t dataId);

    std::size_t readStream(void* buffer, std::size_t length);

    bool underflow();

    virtual bool writeRecordHeader(Stream* stream) = 0;

protected:
    using buffer_type       = utils::circular_buffer<uint8_t>;
    using builder_map_type  = std::map<uint64_t, detail::MsgBuilder*>;
    Stream*                     _stream;
    Stream*                     _recordStream;
    buffer_type                 _buffer;
    builder_map_type            _builders;
    std::size_t                 _min_size;
    std::size_t                 _max_size;
    uint8_t                     _temp[8192];
    uint8_t                     _write_buffer[8192];
    utils::buffer_reader*       _reader;
    bool                        _closed;
    bool                        _byteSwap;
    uint64_t                    _totalByteCount;
    uint64_t                    _tell;
    uint64_t                    _size;
    // mutable std::mutex          _streamMutex;
};

template<typename _Msg>
bool StreamDecoder::material(uint64_t dataId)
{
    detail::MsgBuilder* builder = new detail::MsgBuilderImpl<_Msg>();
    auto ret = _builders.insert(std::make_pair(dataId, builder));
    if (!ret.second) {
        delete builder;
        return false;
    }
    if (_min_size > _Msg::min_size()) {
        _min_size = _Msg::min_size();
    }
    if (_max_size < _Msg::max_size()) {
        _max_size = _Msg::max_size();
    }
    return true;
}

}
}

#endif //__STAR_SDK_DECODER_H
