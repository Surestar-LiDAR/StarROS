/**
 * @author      : John 
 * @date        : 2017-03-07
 */

#ifndef __STAR_SDK_LOGGER_H
#define __STAR_SDK_LOGGER_H

#include <star/Star.h>

#include <cstdarg>
#include <string>

namespace ss {

/**
 * 日志底层实现的接口类
 */
class __star_export LoggerPrivate {
    public:
    virtual void vlog(const char *file, size_t filelen, const char *func, size_t funclen, long line, int level, const char *format, va_list args) = 0;
};

/**
 * 日志对象
 * @see Logger_simple
 * @see Logger_zlog
 */
class __star_export Logger {
public:
    typedef enum {
        LEVEL_FATAL = 0,
                LEVEL_ERROR,
                LEVEL_WARN,
                LEVEL_NOTICE,
                LEVEL_INFO,
                LEVEL_DEBUG,
                LEVEL_COUNT
    } level_type;

    /**
     * 默认构造函数，使用默认的底层接口，在Linux下使用Logger_zlog，在Windows下使用Logger_simple
     * @param category 日志的分类，默认为default，在Windows下该参数无效
     */
    explicit Logger(const std::string &category = "default");

    /**
     * 构造函数，指定底层接口
     * @param loggerPrivate 底层接口对象指针，中间件提供Logger_simple和Logger_zlog
     * @note 对象传递给Logger后其内存由Logger管理。
     */
    explicit Logger(LoggerPrivate *loggerPrivate);

    ~Logger();

    /**
     * 打印日志，应用层不建议调用该函数，除非有自定义的日志等级。
     * @param file  打印日志的文件名，可以使用__FILE__内置宏
     * @param filelen  文件名长度，可以使用sizeof(__FILE__) - 1
     * @param func  打印日志的函数名，可以使用__FUNCTION__或者__func__内置宏
     * @param funclen 函数名长度，可以使用sizeof(__FUNCTION__) - 1或者sizeof(__func__) - 1
     * @param line  打印日志的代码所在行，可以使用__LINE__内置宏
     * @param level 日志等级@see level_type
     * @param format 日志日志输出格式，等同于printf的日志输出格式
     * @param ...    其他参数，等同于printf的其他参数
     */
    void log(const char *file, size_t filelen, const char *func, size_t funclen, long line, int level, const char *format, ...);

    /**
     * 打印日志，应用层不建议调用该函数
     * @param file  打印日志的文件名，可以使用__FILE__内置宏
     * @param filelen  文件名长度，可以使用sizeof(__FILE__) - 1
     * @param func  打印日志的函数名，可以使用__FUNCTION__或者__func__内置宏
     * @param funclen 函数名长度，可以使用sizeof(__FUNCTION__) - 1或者sizeof(__func__) - 1
     * @param line  打印日志的代码所在行，可以使用__LINE__内置宏
     * @param level  日志等级@see level_type
     * @param format 日志日志输出格式，等同于vprintf的日志输出格式
     * @param args   其他参数，等同于vprintf的其他参数
     */
    void vlog(const char *file, size_t filelen, const char *func, size_t funclen, long line, int level, const char *format, va_list args);

    /**
     * 打印LEVEL_FATAL等级的日志
     */
    void fatal(const char *file, size_t filelen, const char *func, size_t funclen, long line, const char *format, ...);

    /**
     * 打印LEVEL_ERROR等级的日志
     */
    void error(const char *file, size_t filelen, const char *func, size_t funclen, long line, const char *format, ...);

    /**
     * 打印LEVEL_WARN等级的日志
     */
    void warn(const char *file, size_t filelen, const char *func, size_t funclen, long line, const char *format, ...);

    /**
     * 打印LEVEL_NOTICE等级的日志
     */
    void notice(const char *file, size_t filelen, const char *func, size_t funclen, long line, const char *format, ...);

    /**
     * 打印LEVEL_INFO等级的日志
     */
    void info(const char *file, size_t filelen, const char *func, size_t funclen, long line, const char *format, ...);

    /**
     * 打印LEVEL_DEBUG等级的日志
     */
    void debug(const char *file, size_t filelen, const char *func, size_t funclen, long line, const char *format, ...);

#if 0
    void vlog(const std::string &file, const std::string &func, long line, int level, const char *format, va_list args);
    void log(const std::string &file, const std::string &func, long line, int level, const char *format, ...);
    void log(const char *file, size_t filelen, const char *func, size_t funclen, long line, int level, const std::string &message);
    void log(const std::string &file, const std::string &func, long line, int level, const std::string &message);
    void fatal(const char *file, size_t filelen, const char *func, size_t funclen, long line, const std::string &message);
    void error(const char *file, size_t filelen, const char *func, size_t funclen, long line, const std::string &message);
    void warn(const char *file, size_t filelen, const char *func, size_t funclen, long line, const std::string &message);
    void notice(const char *file, size_t filelen, const char *func, size_t funclen, long line, const std::string &message);
    void info(const char *file, size_t filelen, const char *func, size_t funclen, long line, const std::string &message);
    void debug(const char *file, size_t filelen, const char *func, size_t funclen, long line, const std::string &message);
#endif

    /**
     * 打印日志，日志输出时不带文件、函数和行数信息，不建议应用层调用该函数
     * @param level 日志等级
     * @param format 日志输出格式，等同于printf的日志输出格式
     * @param ...    其他参数，等同于printf的其他参数
     */
    void log(int level, const char *format, ...);

    /**
     *
     * 打印日志，日志输出时不带文件、函数和行数信息，不建议应用层调用该函数
     * @param level 日志等级
     * @param format 日志日志输出格式，等同于vprintf的日志输出格式
     * @param args   其他参数，等同于vprintf的其他参数
     */
    void vlog(int level, const char *format, va_list args);

    /**
     * 打印LEVEL_FATAL等级的日志
     */
    void fatal(const char *format, ...);

    /**
     * 打印LEVEL_ERROR等级的日志
     */
    void error(const char *format, ...);

    /**
     * 打印LEVEL_WARN等级的日志
     */
    void warn(const char *format, ...);

    /**
     * 打印LEVEL_NOTICE等级的日志
     */
    void notice(const char *format, ...);

    /**
     * 打印LEVEL_INFO等级的日志
     */
    void info(const char *format, ...);

    /**
     * 打印LEVEL_DEBUG等级的日志
     */
    void debug(const char *format, ...);

#if 0
    void log(int level, const std::string &message);
    void fatal(const std::string &message);
    void error(const std::string &message);
    void warn(const std::string &message);
    void notice(const std::string &message);
    void info(const std::string &message);
    void debug(const std::string &message);
#endif

#if 0
    void f(const char *tag, const char *format, ...);
    void f(const std::string &tag, const std::string &message);

    void e(const char *tag, const char *format, ...);
    void e(const std::string &tag, const std::string &message);

    void w(const char *tag, const char *format, ...);
    void w(const std::string &tag, const std::string &message);

    void n(const char *tag, const char *format, ...);
    void n(const std::string &tag, const std::string &message);

    void i(const char *tag, const char *format, ...);
    void i(const std::string &tag, const std::string &message);

    void d(const char *tag, const char *format, ...);
    void d(const std::string &tag, const std::string &message);
#endif

    //定义预处理宏，简化一些调试输出
#if _DEBUG
    //调试模式输出__FILE__， __FUNCTION__， __LINE__
#define tfatal(format, ...) \
    fatal(__FILE__, sizeof(__FILE__) - 1, __FUNCTION__, sizeof(__FUNCTION__) - 1, __LINE__, format, ##__VA_ARGS__)

#define terror(format, ...) \
    error(__FILE__, sizeof(__FILE__) - 1, __FUNCTION__, sizeof(__FUNCTION__) - 1, __LINE__, format, ##__VA_ARGS__)

#define twarn(format, ...) \
    warn(__FILE__, sizeof(__FILE__) - 1, __FUNCTION__, sizeof(__FUNCTION__) - 1, __LINE__, format, ##__VA_ARGS__)

#define tnotice(format, ...) \
    notice(__FILE__, sizeof(__FILE__) - 1, __FUNCTION__, sizeof(__FUNCTION__) - 1, __LINE__, format, ##__VA_ARGS__)

#define tinfo(format, ...) \
    info(__FILE__, sizeof(__FILE__) - 1, __FUNCTION__, sizeof(__FUNCTION__) - 1, __LINE__, format, ##__VA_ARGS__)

#define tdebug(format, ...) \
    debug(__FILE__, sizeof(__FILE__) - 1, __FUNCTION__, sizeof(__FUNCTION__) - 1, __LINE__, format, ##__VA_ARGS__)
#else
#define tfatal(format, ...) \
    fatal(format, ##__VA_ARGS__)

#define terror(format, ...) \
    error(format, ##__VA_ARGS__)

#define twarn(format, ...) \
    warn(format, ##__VA_ARGS__)

#define tnotice(format, ...) \
    notice(format, ##__VA_ARGS__)

#define tinfo(format, ...) \
    info(format, ##__VA_ARGS__)

#define tdebug(format, ...) \
    debug(format, ##__VA_ARGS__)
#endif

private:
    LoggerPrivate   *_impl;
};

}

#endif //__STAR_SDK_LOGGER_H
