/**
 * @author   lucb
 * @date     2020/1/8
 */

#ifndef __STAR_SDK_LAS_WRITER_H
#define __STAR_SDK_LAS_WRITER_H

#include <star/Star.h>

#include <star/fmt/Point.h>

#include <star/utils/buffer.h>
#include <star/utils/buffer_writer.h>
#include <star/Configure.h>

#include <fstream>
#include <iostream>
#include <cstddef>

namespace ss {
namespace fmt {

/**
 * 输出文件写入
 * @note: echo输出参数和mirror输出参数不从配置文件中读取，需要应用层手动设置。
 */
class __star_export Writer {
public:
    constexpr static uint32_t ECHO1 = 0x0001u << 0u;
    constexpr static uint32_t ECHO2 = 0x0001u << 1u;
    constexpr static uint32_t ECHO3 = 0x0001u << 2u;
    constexpr static uint32_t ECHO4 = 0x0001u << 3u;
//    const static uint32_t MIRROR1 = 0x0010u << 1u;
//    const static uint32_t MIRROR2 = 0x0010u << 2u;
//    const static uint32_t MIRROR3 = 0x0010u << 3u;
//    const static uint32_t MIRROR4 = 0x0010u << 4u;
    constexpr static uint32_t TZERO = 0x1000u << 0u;
    constexpr static uint32_t UTC   = 0x1000u << 1u;

    using buffer_type = utils::mutable_buffer<uint8_t>;
    using buffer_writer = utils::native_buffer_writer<buffer_type>;

    Writer() noexcept;
    virtual ~Writer() noexcept = default;

    virtual void setup(const Configure& configure);

    void setFlags(uint32_t flags);
    void setDeviceInformation(int type, uint32_t data, uint32_t version);

    virtual void writeHeader();
    virtual void writePoints(const SHOTS_CALCOUT_S& shot);

    virtual void open(const std::string& path, const std::string& fileName);
    virtual void open(const std::string& path) = 0;
    virtual std::string suffix() const = 0;

    virtual bool filter(const Point& point) const;
    virtual std::size_t write(const Point& point) = 0;
    virtual void close() = 0;

protected:
    int         _deviceType;
    uint32_t    _deviceDate;
    uint32_t    _deviceVersion;
    uint32_t    _flags;
};

class __star_export FileWriter : public Writer {
public:
    FileWriter() noexcept = default;

    explicit FileWriter(std::ios::openmode mode) noexcept;
    explicit FileWriter(const std::string& path, std::ios::openmode mode) noexcept;
//    explicit FileWriter(std::ostream& os) noexcept;

    virtual ~FileWriter() noexcept;

    void open(const std::string& path) override;

    void close() override;

protected:
    std::ios::openmode _openmode;
    std::ofstream _os;
};

}
}


#endif //__STAR_SDK_LAS_WRITER_H
