/**
 * @author   lucb
 * @date     2019/12/31
 */

#ifndef __STAR_SDK_READER_H
#define __STAR_SDK_READER_H

#include <star/Star.h>
#include <star/Packet.h>

#include <star/utils/mutex.h>
#include <star/Configure.h>
#include <vector>

typedef struct _DecPara DecPara_S;

namespace ss {
class Stream;

namespace msg {
class Reader;
}

/**
 * 解码类
 */
class __star_export Reader {
private:
    constexpr static uint32_t ISF_FORMAT = 0x100000u;
    constexpr static uint32_t IMP_FORMAT = 0x200000u;
public:
    //数据格式
    constexpr static uint32_t ISF_VER_4     = ISF_FORMAT | 0x0400u;  /// ISF 4.0格式
    constexpr static uint32_t ISF_VER_4_1   = ISF_FORMAT | 0x0410u;  /// ISF 4.1格式，为带StageAngle的ISF 4.0
    constexpr static uint32_t ISF_VER_5     = ISF_FORMAT | 0x0500u;  /// ISF 5.0格式
    constexpr static uint32_t IMP_VER_1     = IMP_FORMAT | 0x0100u;  /// IMP 1.0格式，为旧版本的IMP格式，文件头为IMP Header
    constexpr static uint32_t IMP_VER_3     = IMP_FORMAT | 0x0200u;  /// IMP 3.0格式，为新规划的IMP格式，文件头为寄存器表

    //输出的数据包，@see Packet in Packet.h
    using Packets = Packet;

    /**
     * 默认构造函数
     */
    Reader();

    /**
     * 构造函数，制定读取的数据格式
     * @param formatVersion 读取的数据格式
     */
    explicit Reader(uint32_t formatVersion);

    ~Reader();

    /**
     * 打开Reader，open函数后创建Decoder对象解码
     * @return 成功返回true，失败返回false
     */
    bool open();

    /**
     * 读取stream的数据，如果Reader已经打开了一个Stream，则会先关闭旧的decoder创建一个新的
     * @param stream 要读取的stream
     * @return 成功返回true，失败返回false
     */
    bool open(Stream* stream);

    /**
     * 关闭Reader，关闭后可以重新打开
     */
    void close();

    /**
     * 打开一个新的Stream，如果已经创建了Decoder则不会创建新的
     * @param stream 要读取stream
     * @return 成功返回true，失败返回false
     */
    bool openNewStream(Stream* stream);

    /**
     * 判断stream是否读取结束或者是否存在错误
     * @return 当文件读取到结尾，Stream出错异常关闭时，返回true，否则返回false
     */
    bool eof() const;

    /**
     * 跳转到制定的位置，只在文件流中有效
     * @param pos 指定的位置
     * @return 成功则返回true，否则返回false
     */
    bool seek(int64_t pos);

    /**
     * 获取当前读取的位置，文件流为文件指针的位置，其他为累计读取的数据量
     * @return 读取的位置
     */
    int64_t tell() const;

    /**
     * 获取当前流的大小，只对文件流有效，其他流返回0
     * @return 文件流返回文件的大小，其他返回0
     */
    int64_t size() const;

    /**
     * 设置是否时阻塞模式工作，已失效，设置任何值都无效
     * @param blockMode 是否使用阻塞模式，已经无效
     */
    void setBlockMode(bool blockMode);

    /**
     * 打开中间测试数据输出，主要是处理IMP文件的慢数据输出
     * @param path 输出的路径
     * @param parameters 使能选项
     */
    void openDebugs(const std::string& path, const cfg::DebugOutput& parameters);

    /**
     *  设置解码的格式
     * @param formatVersion 解码格式，参见格式说明
     */
    void setFormatVersion(uint32_t formatVersion);

    /**
     * 开始录像
     * @param stream 写入录像数据的stream，该stream设置后由reader管理，用户不需要释放
     * @return 开始成功则返回true，否则返回false
     * @note 录像不支持seek操作
     */
    bool startRecord(Stream* stream);

    /**
     * 开始录像，录像数据存放在指定路径的文件中
     * @param path 指定的文件路径
     * @return 开始成功则返回true，否则返回false
     * @note 录像不支持seek操作
     */
    bool startRecord(const std::string& path);

    /**
     * 停止录像，保存录像数据，释放相关资源。
     */
    void stopRecord();

    /**
     * 获取参考的时间戳，用于解码时时间戳的计算，单位us，一般为从数据文件中读取到的时间值
     * @return 参考的时间戳
     */
    int64_t timestamp() const;

    /**
     * 设置参考的时间戳，用于解码时时间戳的计算
     * @param lastSecond 设置的时间戳，单位us，一般为从数据文件中读取到的时间值
     */
    void setTimestamp(int64_t lastSecond);

#if 0
    void setMetadata(const ss::msg::Imp_MetaData_S& metadata);
	
    const ss::msg::Imp_MetaData_S& metadata() const;
    ss::msg::Imp_MetaData_S& metadata();
#endif
    bool waitMetadata(); //等待metadata读取完成，如果数据流不包含metadata，不要调用该函数
    bool waitMetadata(Configure& configure);
    void setMetadata(const Configure& metadata);

    void updateConfigure(Configure& configure) const;
	
    Packet getPacket();
    Packets getPackets();

protected:
    void setStream(Stream* stream);
    void waitReaderIdle();

private:
    uint32_t				        _format;
	mutable std::recursive_mutex	_reader_mutex;
    msg::Reader*		            _reader;
    bool                            _busy;
    int64_t                         _timestamp;
};

}

#endif //__STAR_SDK_READER_H
