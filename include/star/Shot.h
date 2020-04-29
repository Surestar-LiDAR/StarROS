/**
 * @author   lucb
 * @date     2020/3/2
 */

#ifndef __STAR_SDK_MSG_SHOT_H
#define __STAR_SDK_MSG_SHOT_H

#include <star/Lidar.h>

namespace ss {

struct Point2d {
    double x = 0;
    double y = 0;
};

struct Point3d {
    double x = 0;
    double y = 0;
    double z = 0;
};

struct LasPulse_S {
    bool flag = false;             //zks: ??? for what ???
    uint16_t m_uInt = 0; ///< intensity
    uint16_t m_uInt_wide = 0;
    float m_fRange = 0;          ///<波形数据
    float m_fRange_T0 = 0;          ///<波形数据
    float m_cPulseWidth = 0;  ///<波形宽度
    // unsigned int32_t m_cPulseWidth ;  ///<波形宽度
    uint32_t riseEdge = 0;  //零位信号脉冲计数
};

struct LasShot_S {
    bool flag = false; //标志是否有回波数据
    bool flagSL = false;
    uint16_t dataID = 0;
    uint16_t lidarID = 0;     //激光器ID   add by scofield zhang 2019 10 17
    uint16_t planeNum = 0;
    lidar::MirrorNumber mirrorNumb = lidar::zeroAngSignal;   //zeroAngSignal 第4回波 riseEdge 表示 脉冲计数 count值
    int32_t angleArea = 0;
    uint32_t m_tzero = 0; ///<
    int32_t data_grade = 0;
    int32_t mpia_sel = 0;
    double m_dAngleX = 0;     ///<电机扫描角X
    double turnAngle = 0;   //转台角度
    double afa = 0;
    double bta = 0;
    double utcTime = 0;    ///<utc时间(单位:周秒)  add by scofield zhang 2019 10 17
    LasPulse_S m_pulse[4]; ///<回波数据
};

#define PPS_STAMP_DEFAULT (0x80000000) //max T0,2^31, 2147483648
#define PPS_DIFF_DEFAULT  (5000000)
#define UTC_STAMP_DEFAULT (0.0)

struct PosPPS_S {
    uint32_t ppsStamp = PPS_STAMP_DEFAULT;
    int32_t diffPpsStamp = PPS_DIFF_DEFAULT; //zks: alway positive, if negative, something wrong!
    double utcStamp = UTC_STAMP_DEFAULT;
};

/**
* \struct  SLasResolvedPulse
* @n
* 解算后回波数据。通过解算，生成大地坐标后的回波数据及其相关辅助数据
*/
struct LasResolvedPulse {
    int stripNb = 0;		///< 航带号
    double X = 0;		///< X坐标
    double Y = 0;		///< Y坐标
    double Z = 0;		///< Z坐标
};

/**
* \struct  SLasResolvedShot
* @n
* 一束扫描激光所获取数据解算后结果存放数据结构
*/
typedef struct LasResolvedShot {
    LasShot_S lasShot;
    double stageangle = 0;   ////转台角度信息，通过内插获取
    double bubble_x = 0;
    double bubble_y = 0;
    double m_dGPSTime = 0;	///<GPS时间(单位:周秒)
    double pos[6] = {0};	///<GPS坐标(米)，后两位值记录经纬度(弧度)。内插
    double rph[3]  = {0};	///<IMU姿态roll,pitch,heading(弧度)。内插
    LasResolvedPulse m_pulse[4]; ///<解算后的回波数据
} SHOTS_CALCOUT_S ; // SHOTS_CALCOUT_S ;


struct PosMsg_S {
    double gpstime = 0;
    double heading = 0;
    double pitch = 0;
    double roll = 0;
    double latitude = 0;
    double lontitude = 0;
    double height = 0;
    Point3d velocity;
};

//指北针
struct Narrow_S {
    double narrowAngle = 0;
    double turnAngle = 0;
};

}

#endif //__STAR_SDK_MSG_SHOT_H
