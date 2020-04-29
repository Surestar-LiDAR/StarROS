/**
 * @author      : John 
 * @date        : 2016-05-27
 */

#ifndef __DATE_TIME_H
#define __DATE_TIME_H

#include <star/Star.h>

#include <string>
#include <cstdint>

namespace ss {

/**
 * 日期时间对象，处理系统的日期时间
 */
class __star_export DateTime {
public:
    /**
     * 构造函数，初始化为当前时间
     */
    DateTime();

    /**
     * 构造函数，设置时间为time
     * @param time epoch毫秒
     */
    explicit DateTime(long long time);

    /**
     * 构造函数
     * @param time 时间字符串
     * @param format 时间字符串的格式，参考parse函数
     */
    DateTime(const std::string &time, const std::string &format);

    /**
     * 构造函数
     * @param time 时间字符串
     * @param length 时间字符串长度
     * @param format 格式字符串
     * @param formatLength 格式字符串长度
     */
    DateTime(const char* time, size_t length, const char* format, size_t formatLength);

    DateTime(int year, int month, int day, int hour, int minute, int second, int millisecond = 0);

    /**
     * 设置时间
     * @param time epoch毫秒
     */
    void setTime(int64_t time);

    /**
     * 获取epoch毫秒
     * @return 从1970年1月1日00:00:00:000（CUT）开始经过的毫秒数
     */
    int64_t time() const;

    void add(int year, int month, int day, int hour, int minute, int second, int millisecond = 0);

    /**
     * 设置时间
     * @param year 公元的年
     * @param month 月，取值从1~12
     * @param day   日，取值1~31
     * @param hour  时，取值0~23
     * @param minute 分，取值0~59
     * @param second 秒，取值0~60
     * @param millisecond 毫秒，取值0~999
     */
    void setTime(int year, int month, int day, int hour, int minute, int second, int millisecond = 0);

    /**
     * 设置时间
     * @param hour
     * @param minute
     * @param second
     * @param millisecond
     */
    void setTime(int hour, int minute, int second, int millisecond = 0);

    /**
     * 获取日期的年
     * @return 返回公元的年
     */
    int year() const;

    /**
     * 获取日期的月
     * @return 返回一年中的月份，范围1~12
     */
    int month() const;

    /**
     * 获取日期的日
     * @return 返回一个月中的日，范围1~31
     */
    int day() const;

    int dayOfYear() const;

    int dayOfWeek() const;

    /**
     * 获取时间的时
     * @return 返回一天中的时，范围0~23
     */
    int hour() const;

    /**
     * 获取时间的分
     * @return 返回一小时中的分，范围0~59
     */
    int minute() const;

    /**
     * 获取时间的秒
     * @return 返回一分钟中的秒，范围0~60(润秒)
     */
    int second() const;

    /**
     * 获取时间的毫秒
     * @return 获取一秒钟的毫秒，范围0~999
     */
    int millisecond() const;

    /**
     * 获取当前系统毫秒
     * @return 从1970年1月1日00:00:00:000（CUT）开始到当前时间的毫秒数
     */
    static int64_t epoch();

    /**
     * 系统运行时间
     * @return 启动启动运行的毫秒数
     */
    static int64_t tickTime();

    /**
     * 解析时间字符串
     * @param time 时间字符串
     * @param length 时间字符串长度
     * @param format 格式字符串
     * @param formatLength 格式字符串长度
     *
     * 格式字符串定义
     * y     年，按照实际长度如192年，2007年
     * yyyy  年，占四个字符，如0129年
     * M     月，按照实际长度，如1月
     * MM    月，占两个字符，如01月
     * d     日，按照实际长度，如1日
     * dd    日，占两个字符，如01日
     * h     时，按照实际长度，如1时
     * hh    时，占两个字符，如01时
     * m     分，按照实际长度，如1分
     * mm    分，占两个字符，如01分
     * s     秒，按照实际长度，如1秒
     * star    秒，占两个字符，如01秒
     * S     毫秒，按照实际长度，如1毫秒
     * SSS   毫秒，占三个字符，如001毫秒
     */
    void parse(const char* time, size_t length, const char* format, size_t formatLength);

    /**
     * 解析时间字符串，等价于parse(time.c_str(), time.length(), format.c_str(), format.length())
     */
    void parse(const std::string &time, const std::string &format);

    /**
     * 将时间格式化为字符串
     * @param dist 输出字符数组地址
     * @param length 输出字符数组的长度
     * @param fmt    格式化字符串
     * @param formatLength 格式化字符串长度
     * @return 实际生成的字符串长度
     */
    size_t format(char* dist, size_t length, const char* fmt, size_t formatLength);

    /**
     * 将时间格式化为字符串
     * @param fmt
     * @param formatLength
     * @return 格式化后的时间字符串
     */
    std::string format(const char* fmt, size_t formatLength);

    /**
     * 将时间格式化为字符串
     * @param fmt
     * @return 格式化后的时间字符串
     */
    std::string format(const std::string &fmt);

private:
    int _millisecond;               /* millisecond [0-999]*/
    int _second;                    /* Seconds.    [0-60] (1 leap second) */
    int _minute;                    /* Minutes.    [0-59] */
    int _hour;                      /* Hours.      [0-23] */
    int _day;                       /* Day.        [1-31] */
    int _month;                     /* Month.      [0-11] */
    int _year;                      /* Year        .      */
};

}

#endif //__DATE_TIME_H
