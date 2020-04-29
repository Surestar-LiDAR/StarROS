#ifndef __STAR_SDK_CAL_TWO_AXIS_LIDAR_COOR_H
#define __STAR_SDK_CAL_TWO_AXIS_LIDAR_COOR_H

#include <star/Configure.h>
#include <star/Shot.h>

namespace ss {
namespace calc {


//typedef struct{
//    double angle;
//    double dIx;
//    double dIy;
//}UR_Inclt_S;
//
//typedef struct _ot_S {
//    float Hs; //设备高 单位m
//    float x0, y0,z0 ; //定向点坐标 单位m
//    float x1, y1,z1 ; //后视点坐标 单位m
//    float turnAngle;  //转台角度  单位度
//    _ot_S() {
//        Hs=0.0;
//        x0=0.0 ;
//        y0=0.0;
//        z0=0.0;
//        x1=0.0;
//        y1=0.0;
//        z1=0.0;
//        turnAngle=0.0;
//    }
//}PRE_BackCenter_S ;
//
//typedef struct _gps_s {
//
//    float cMeridian; //中央子午线经度 单位度
//    float B;  //纬度 单位度
//    float L;   //经度 单位度
//    float H;  //高度 单位m
//    _gps_s() {
//        cMeridian=0.0;
//        B=0.0;
//        L=0.0;
//        H=0.0;
//    }
//}PRE_CenterPoint_S;
//
////指北针
//typedef struct {
//    double narrowAngle;
//    double turnAngle;
//} Narrow_S;

class Interpolation;

class CalTwoAxisLidarCoor
{
    //POS模块的选择
    struct POSModuleSelect {
        bool isUTCInterp;        //UTC内插
        bool   isGeoConvert;   //北东天坐标
        bool  isProjctConvert;  //投影坐标
        bool  isUAGPSConvert;  //UA设备GPS定位
        bool  isOrientConvert;    //UA设备的后视定向
    };

public:
    CalTwoAxisLidarCoor();
    ~CalTwoAxisLidarCoor();
	bool setup(const ss::Configure& configure ) ;

    int calXYZ(SHOTS_CALCOUT_S *currshot) ;

protected:
    int angleRotate(double turnAngle,double bubbleX,double bubbleY) ;
    int cal_uaXYZ(SHOTS_CALCOUT_S *currshot) ;
    int cal_slXYZ(SHOTS_CALCOUT_S *currshot);
    int orientReferCoor(SHOTS_CALCOUT_S *currshot) ;
    int GpsNarrowCoor(SHOTS_CALCOUT_S *currshot);

private:
//    int interpInclt(SHOTS_CALCOUT_S *currshot);

    Interpolation* m_inter;

    ss::cfg::CommonCalibParams  m_commonCalibParams;
    ss::cfg::UaCalibParams      m_uaCalibParams;
    ss::cfg::StageCalibParams  m_stageCalibParams;
    ss::cfg::GpsUAPosition m_gpsUAPosition;
    ss::cfg::OrientUA m_orientUA;

   double m_roll,m_pitch,m_heading;
    Narrow_S m_narrow_;
    double m_turnAngle;
    double m_narrowAngle;

    POSModuleSelect  m_posSelect;
     int  m_deviceType;
};

}
}

#endif //__STAR_SDK_CAL_TWO_AXIS_LIDAR_COOR_H
