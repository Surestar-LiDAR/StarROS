/**
 * @author      : John
 * @date        : 2017-05-09
 */

#ifndef __SS_IO_DEVICE_H
#define __SS_IO_DEVICE_H

#include <star/Star.h>

#include <string>

#if defined(_WIN32)
#include <Windows.h>
#include <winioctl.h>
#else
#include <sys/fcntl.h>
#endif

#include <star/Stream.h>

namespace ss {

class __star_export IODevice : public Stream {
public:
#if defined(_WIN32)
    typedef HANDLE    handle_t;
    typedef u_long    ioctl_cmd_t;
#else
    typedef int     handle_t;
    typedef int     ioctl_cmd_t;
#endif
    IODevice();
    virtual ~IODevice() noexcept;

    /**
     * 打开设备文件
     * @param device 设备文件路径
     * @return 成功返回true, 失败返回false
     */
    bool open(const std::string &device);

    void   close() override;
    ssize_t read(char* buffer, size_t length) override;
    ssize_t write(const char* buffer, size_t length) override;

    bool seek(int64_t pos) override;

    int64_t tell() const override;

    int64_t size() const override;

    bool    ioctl(unsigned long request, const void *data, size_t length);
    ssize_t ioctl(unsigned long request, const void *data, size_t length, void *output, size_t olen);


    virtual ssize_t available();

    bool opened() const;

    bool eof() override;

    bool good() override;

    handle_t nativeHandle();
    handle_t handle();

private:
    void closeDevice();

protected:
    handle_t    _handle;
    bool        _opened;
    int64_t     _rdsize;
    int64_t     _wrsize;
};

}

#endif //__SS_IO_DEVICE_H
