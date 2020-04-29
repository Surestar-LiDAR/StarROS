/**
 * @author  lucb
 * @date    2019/12/9.
 */

#ifndef __STAR_SDK_STREAM_H
#define __STAR_SDK_STREAM_H

#include <star/Star.h>

#include <cstddef>
#include <cstdint>

namespace ss {

class Stream {
public:
    virtual ~Stream() noexcept = default;

//    virtual bool open() = 0;

    virtual void close() = 0;

    virtual ssize_t read(char* buffer, std::size_t length) = 0;

    virtual ssize_t write(const char* buffer, std::size_t length) = 0;

    virtual bool eof() = 0;

    virtual bool good() = 0;

    virtual bool seek(int64_t pos) = 0;

    virtual int64_t tell() const = 0;

    virtual int64_t size() const = 0;
};

}

#endif //__STAR_SDK_STREAM_H
