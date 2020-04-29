/**
 * @author      : John 
 * @date        : 2015-08-12 15:15
 */

#ifndef __CORE_SERIAL_H
#define __CORE_SERIAL_H

#include <star/Star.h>
#include "IODevice.h"

namespace ss {

/**
 * 串口类
 */
class __star_export Serial : public IODevice {
public:
    /**
     * 停止位
     */
    typedef enum stop_type {
        SERIAL_STOPBITS_1   = 1,
        SERIAL_STOPBITS_2   = 2,
        SERIAL_STOPBITS_1_5 = 3,
    } stopbits_t;

    /**
     * 校验位
     */
    typedef enum parity_type {
        SERIAL_PARITY_NONE  = 0,
        SERIAL_PARITY_EVEN  = 1,
        SERIAL_PARITY_ODD   = 2,
        SERIAL_PARITY_MARK  = 3,
        SERIAL_PARITY_SPACE = 4,
    } parity_t;

    /**
     * 构造函数
     */
    Serial();

    /**
     * 析构函数，会自动调用close关闭串口
     */
    ~Serial() noexcept;

private:
    /**
     * 禁用拷贝
     */
    Serial(const Serial &);
    Serial &operator=(const Serial &);

public:

    /**
     * 配置串口
     * @param baud 波特率
     * @param databits 数据位
     * @param stopbits 停止位
     * @param parity 校验位
     */
    bool setup(int baud = 115200, int databits = 8, stopbits_t stopbits = SERIAL_STOPBITS_1, parity_t parity = SERIAL_PARITY_NONE) ;

    /**
     * 串口读
     * @param buffer 
     * @param length
     * @return 返回传输的数据长度，小于0表示失败
     */
    ssize_t read(char* buffer, size_t length) override;

    /**
     * 串口写
     * @param buffer
     * @param length
     * @return 返回传输的数据长度，小于0表示失败
     */
    ssize_t write(const char* buffer, size_t length) override ;

    /**
     * 获取串口接收缓存中的数据
     * @return 可读的数据长度，小于0表示失败
     */
    ssize_t available() override;

    /**
     * 获取串口的波特率
     * @return 波特率
     */
    int baud() const;
};

}

#endif //__CORE_SERIAL_H
