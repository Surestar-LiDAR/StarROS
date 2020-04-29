/**
 * @author   lucb
 * @date     2020/1/2
 */

#ifndef __STAR_SDK_IO_MEMORY_H
#define __STAR_SDK_IO_MEMORY_H

#include <star/Star.h>
#include <star/Stream.h>
#include <star/utils/circular_buffer.h>

namespace ss {

/**
 * IOMemory允许从一端写入数据，另一端读取数据，
 * 配合Reader使用时能实现自定义的数据输入
 */
class __star_export IOMemory  : public Stream {
public:
    void close() override;

    ssize_t read(char* buffer, std::size_t length) override;

    ssize_t write(const char* buffer, std::size_t length) override;

    bool eof() override;

    bool good() override;

    bool seek(int64_t pos) override;

    int64_t tell() const override;

    int64_t size() const override;

private:
    ss::utils::circular_buffer<char>    _memory;
};

}

#endif //__STAR_SDK_IO_MEMORY_H
