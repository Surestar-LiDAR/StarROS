/*
 * @author      : John
 * @date        : 2016-3-1
 */

#ifndef __SURE_STAR_SOCKET_H
#define __SURE_STAR_SOCKET_H

#include <star/Star.h>
#include "Stream.h"

#include <string>

struct sockaddr;

namespace ss {

/**
 * 套接字端点类型基类
 */
class __star_export SocketEndpoint {
public:
    /**
     * 构造函数
     */
    SocketEndpoint() noexcept;

    /**
     * 构造函数
     * @param family
     */
    explicit SocketEndpoint(short family) noexcept;

    /**
     * 获取协议簇
     * @return
     */
    short family() const noexcept;

    /**
     * 获取原始地址类型指针
     * @return 原始套接字地址类型指针
     */
    sockaddr* native() noexcept;

    /**
     * 获取原始地址类型指针
     * @return 原始套接字地址类型指针
     */
    const sockaddr* native() const noexcept;

    /**
     * 原始地址的长度
     * @return 原始地址的长度
     */
    std::size_t length() const noexcept;

private:
    typedef unsigned long int       __ss_align_type;
    constexpr static std::size_t    __ss_size = 128;

    struct __ss_storage_s {
        // uint8_t         __ss_len;         /* total length */
        short           __ss_family;
        char            __ss_padding[__ss_size - sizeof(short) - sizeof(__ss_align_type)];
        __ss_align_type __ss_align;
    } __ss_storage;
};

class __star_export InternetEndpoint : public SocketEndpoint {
public:
    InternetEndpoint();
    InternetEndpoint(const std::string& address, uint16_t port);
    explicit InternetEndpoint(uint16_t port);
};

/**
 * Socket对象，对socket的基础封装。
 */
class __star_export Socket : public Stream {
public:
    typedef int state_t;
    constexpr static state_t NONE = 0;
    constexpr static state_t CLOSED = -2;
    constexpr static state_t OPENED = 1;
    constexpr static state_t BINDED = 2;
    constexpr static state_t CONNECTED = 4;

#if defined(_WIN32)
    typedef int socklen_t;
#ifdef _WIN64
    typedef unsigned __int64 socket_t;
#else
    typedef uint32_t     socket_t;
#endif
//    typedef void *HANDLE;
//    typedef HANDLE handle_t;
    typedef socket_t handle_t;
#else
    typedef int     socket_t;
    typedef socket_t handle_t;
#endif
    Socket();

    /**
     * 构造函数
     * Socket构造函数不会执行socket创建套接字，创建套接字应该执行Socket::socket函数
     * @param family socket的family
     * @param type socket的type
     * @param protocol socket的protocol
     */
    Socket(int family, int type, int protocol);

    /**
     * 析构函数
     * 析构函数会执行close关闭当前的socket（如果socket没有关闭）
     */
    virtual ~Socket() noexcept;

private:
    //禁用拷贝
    Socket(const Socket &);

    Socket &operator=(const Socket &);

public:

    Socket(Socket&& other) noexcept;
    Socket& operator=(Socket&& other) noexcept;

    static Socket udp();
    static Socket tcp();

    /**
     * 创建socket
     */
    bool socket();

    /**
     * 关闭
     */
    void close() override ;

    /**
     * 绑定socket
     * @param address 绑定地址
     */
    bool bind(const SocketEndpoint& address);

    /**
     * 监听socket
     * @param count
     */
    bool listen(std::size_t count);
	
    /**
     * 连接到服务器
     * @param address 服务器地址
     */
    bool connect(const SocketEndpoint& address);

    /**
     * 断开链接
     */
    void disconnect();

    bool accept(Socket& socket) const;

    ssize_t read(char* buffer, size_t length) override;

    ssize_t write(const char* buffer, size_t length) override;

    bool eof() override;

    bool good() override;

    bool seek(int64_t pos) override;

    int64_t tell() const override;

    /**
     * 设置TCP连接的Keep Alive
     * @param keepAlive 控制KeepAlive打开或者关闭
     * @param idle 正常接收到数据到首次发送KeepAlive时的空闲事件，单位ms
     * @param interval 两次检测之间的间隔，单位ms
     * @param count 重复检测的次数，该参数在Windows中无效
     * @note 出错抛出SocketException异常
     */
    bool setKeepAlive(bool keepAlive, unsigned long idle = 1000, unsigned long interval = 1000, int count = 5);

    /**
     * 设置TCP_USER_TIMEOUT
     * @param time 超时时间，单位ms
     * @note 该函数仅在Linux内核2.6.14版本以上有效
     *       出错抛出SocketException异常
     */
    bool setUserTimeout(unsigned long time);

    /**
     * 发送
     * @param buffer 发送缓存
     * @param len 发送长度
     * @param flags 标记，默认为0
     * @return 实际发送的长度
     */
    ssize_t send(const char* buffer, size_t length, int flags = 0);

    /**
    * 接收
    * @param buffer 接收缓存
    * @param len 接收缓存成都
    * @param flags 标记，默认为0
    * @return 实际接收到的长度
    */
    ssize_t receive(char* buffer, size_t length, int flags = 0);

    /**
     * 发送
     * @param address 目的地址
     * @param buffer  发送的数据缓存
     * @param length  发送数据的长度
     * @param flags   标志位
     * @return 实际发送的字节数
     */
    ssize_t send(const SocketEndpoint& address, const char* buffer, size_t length, int flags = 0);

    /**
     * 接收
     * @param address 源地址
     * @param buffer  接收数据的缓存
     * @param length  接收数据的长度
     * @param flags   标志位
     * @return 实际接受的字节数
     */
    ssize_t receive(SocketEndpoint& address, char* buffer, size_t length, int flags = 0);

    int64_t size() const override;

    /**
     * 等同于setsockopt()
     * @param level 等级，当对socket接口设置时，该参数指定为SOL_SOCKET
     * @param option 选项名称
     * @param value 设置参数的地址
     * @param length 设置参数的长度
     */
    bool setOption(int level, int option, const void *value, int length);

    /**
     * 等同getsockopt()
     * @param level 等级，当对socket接口设置时，该参数指定为SOL_SOCKET
     * @param option 选项名称
     * @param value 保存返回值地址
     * @param length 返回值的长度
     */
    bool option(int level, int option, void *value, int &length);

    /**
     * 文件描述符的ioctl()函数
     * @param cmd 命令
     * @param arg 参数
     */
    bool ioctl(long cmd, unsigned long *arg);

    /**
     * 设置socket，设置后默认为打开状态
     * @param sock
     */
    void setSocket(socket_t sock, int family, int type, int protocol);

    /**
     * 获取family
     * @return family
     */
    int family() const;

    /**
     * 获取协议类型
     * @return 返回协议号
     */
    int type() const;

    /**
     * 获取协议号
     * @return 返回协议号
     */
    int protocol() const;

    /**
     * 获取打开状态
     * @return socket已经创建则返回true，否则返回false
     */
    bool opened() const;

    /**
    * 获取绑定状态
    * @return socket已经绑定则返回true，否则返回false
    */
    bool binded() const;

    /**
     * 获取连接状态
     * @return socket已经连接成功则返回true，否则返回false
     */
    bool connected() const;

    /**
    * 获取接收缓存数据长度
    * @return 接收缓存中可用的字节数
    */
    ssize_t available();

    /**
     * Socket阻塞模式设置
     * @param true 设置为非阻塞模式，false 设置为阻塞模式
     */
    bool setNonblocking(bool nonblocking);

    /**
     * Socket 接收发送超时设置
     * @param time 超时时间
     */
    bool setTimeout(unsigned long time);

#if 0
    /**
     * 获取对端IP地址，在connect()或者bind()后有效
     * 获取对端地址，服务器端在调用了bind后执行有效。客户端在connect成功后调用有效
     * @return 对端地址
     */
    InetAddress peerInetAddress() const;

    /**
     * 获取对端地址，在connect()或者bind()后有效
     * @param address[out] 存储获取到的地址， 当为IP协议时返回的为InetAddress地址，当为IPCSocket（Unix only）时应为IPCSocketAddress
     */
    void peerAddress(SocketEndpoint& address) const;

    /**
     * @return 地址返回字符串参见Address::toString方法
     */
    std::string peerAddress() const;

    /**
     * 针对IPv4和IPv6协议，获取本地IP地址
     * @return 本地IP地址
     */
    InetAddress socketInetAddress() const;

    /**
     * 获取本地地址
     * @param address[out] 存储获取到的地址， 当为IP协议时返回的为InetAddress地址，当为IPCSocket（Unix only）时应为IPCSocketAddress
     */
    void socketAddress(SocketEndpoint& address) const;

    /**
     * 返回本机地址的字符串
     */
    std::string socketAddress() const;
#endif

    state_t state() const;

    /**
     * 在Windows下会创建一个HANDLE对象，但并没有绑定到socket
     */
    handle_t nativeHandle();

private:
    void doClose();
	
protected:
    socket_t        _socket;
    int             _family;
    int             _type;
    int             _protocol;

    state_t         _state;
    SocketEndpoint  _endpoint;
	int64_t         _rdsize;
	int64_t         _wrsize;
};

}

#endif //__SURE_STAR_SOCKET_H
