/**
 * @author      : John 
 * @date        : 2017-07-28
 */
#ifndef __STAR_SDK_LOGGER_SIMPLE_H
#define __STAR_SDK_LOGGER_SIMPLE_H

#include <star/Star.h>

#include <star/Logger.h>

#include <cstdio>
#include <cstddef>
#include <string>

#include <star/utils/mutex.h>

namespace ss {

/**
 * 简单的日志底层实现，仅能将日志输出到文件中
 */
class __star_export Logger_simple : public LoggerPrivate {
public:
    /**
     * 构造函数，指定一个打开的文件对象
     * @param file 文件对象，默认为标准输出
     */
    explicit Logger_simple(FILE *file = stdout);

    /**
     * 构造函数，指定输出的文件路径
     * @param file 输出文件的路径
     */
    explicit Logger_simple(const std::string& path);
    Logger_simple(const std::string& path, const std::string& name);

    ~Logger_simple();

    void vlog(const char* file, size_t filelen,
            const char* func, size_t funclen,
            long line, int level,
            const char* format, va_list args);

protected:
    bool createNewFile();
    void closeFile();
private:
    std::string _path;
    std::string _name;
    FILE*       _file;
    std::mutex  _mutex;
    int64_t     _last_timestamp;
    std::size_t _file_count_limit;
};

}

#endif //__STAR_SDK_LOGGER_SIMPLE_H
