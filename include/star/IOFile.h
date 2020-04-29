/**
 * @author   lucb
 * @date     2019/12/10
 */

#ifndef __STAR_SDK_FILE_H
#define __STAR_SDK_FILE_H

#include <star/Star.h>

#include <star/Stream.h>

#include <string>
#include <cstdio>

namespace ss {

class __star_export IOFile : public Stream {
public:
    IOFile();

    explicit IOFile(const std::string& path);
    explicit IOFile(const std::string& path, const std::string& mod);

    void open(const std::string& path, const std::string& mod);

    void close() override;

    ssize_t read(char* buffer, size_t length) override;

    ssize_t write(const char* buffer, size_t length) override;

    bool eof() override;

    bool good() override;

    bool seek(int64_t pos) override;

    int64_t tell() const override;

    int64_t size() const override ;

private:
    FILE*      _file;
};

}

#endif //__STAR_SDK_FILE_H
