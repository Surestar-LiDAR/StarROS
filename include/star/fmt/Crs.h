/**
 * @author   lucb
 * @date     2020/2/14
 */

#ifndef __STAR_SDK_FMT_CRS_H
#define __STAR_SDK_FMT_CRS_H

#include <star/Star.h>
#include <star/fmt/Writer.h>

#include <vector>
#include <star/DateTime.h>

namespace ss {
namespace fmt {

class __star_export CrsBinWriter : public FileWriter{
public:
    CrsBinWriter();
    virtual ~CrsBinWriter();

    void writeHeader() override;

    size_t write(const Point& point) override;

protected:
    void doWriteHeader();
    void updateHeader(const Point& point);

protected:

#pragma pack(push, 1)
    struct Timestamp {
        uint16_t wYear;
        uint16_t wMonth;
        uint16_t wDayOfWeek;
        uint16_t wDay;
        uint16_t wHour;
        uint16_t wMinute;
        uint16_t wSecond;
        uint16_t wMilliseconds;
    };
    //CRS 轮廓断面数据头结构
    struct CsrHeader {
        uint32_t   packFlag ;    //!< 数据包标识: 0xE7E7E7E7
        uint16_t   versions ;    //!< 版本号
        uint16_t   devNum ;      //!< 设备编号
        uint32_t   serialNum ;   //!< 序列号
        uint8_t    devStaFlag ;  //!< 设备状态标识 0正常；非零异常 ； 11 | 11 | 11(激光器状态) | 11(扫描电机状态)
        uint32_t   scanFrq ;     //!< 扫描频率   单位 RPS(转/秒)
        uint32_t   pointFrq ;    //!< 点频       单位 Hz(个/秒)
        uint32_t   angleRes ;    //!< 角度分辨率 单位 千分之一度
        uint64_t   pulseNum ;    //!< DMI脉冲计数
        uint32_t   pointsCount ; //!< 一个轮廓断面总点数
        Timestamp  svrSysTime ;     //!< 断面采集时间，零位时间。
    };

//CRS 点数据结构
    struct CrsPoint {
        float    angle;               //!< 扫描角度
        uint16_t range;      //!< 测距 单位mm
        uint16_t intension;  //!< 反射强度:
    };
#pragma pack(pop)

    struct CrsLine {
        CsrHeader             header;
        std::vector<CrsPoint> points;
    };

private:
    CrsLine     _crsLine;
    uint64_t    _sumSqrCnt;            //总脉冲计算值
    uint32_t    _preSqrCnt;            //上一个脉冲计算值
    double      _timestamp;
    DateTime    _sysTime;
};

}
}

#endif //__STAR_SDK_FMT_CRS_H
