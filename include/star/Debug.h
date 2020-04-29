/**
 * @author   lucb
 * @date     2019/12/19
 */

#ifndef __STAR_SDK_DEBUG_H
#define __STAR_SDK_DEBUG_H

#include <star/Star.h>

#include <cstdio>
#include <string>

namespace ss {

class __star_export Debug {
public:
    Debug();
    explicit Debug(FILE* file);
    explicit Debug(const std::string& path, bool binary);
    virtual ~Debug();

    Debug(const Debug& other) = delete;
    Debug& operator=(const Debug& other) = delete;

    Debug(Debug&& other) noexcept;
    Debug& operator=(Debug&& other) noexcept;

    virtual void set_path(const std::string& path);

    virtual bool open(bool enable);

    void set_enable(bool enable);

    void print(const char* fmt, ...);
private:
    std::string _fileName;
    bool        _binary;
    FILE*       _file;
    bool        _enable;
};

}

#endif //__STAR_SDK_DEBUG_H
