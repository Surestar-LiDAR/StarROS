#ifndef __STAR_SDK_CALC_CAL_COOR_H
#define __STAR_SDK_CALC_CAL_COOR_H

#include <star/Star.h>
#include <star/Lidar.h>

#include <star/calc/PreFilter.h>
#include <star/calc/MultiLevelRegister.h>
#include <star/calc/CalMultiLidarCoor.h>
#include <star/calc/CalibRange.h>
#include <star/calc/CalibZerosAngle.h>
#include <star/calc/CalOneAxisLidarCoor.h>
#include <star/calc/CalTwoAxisLidarCoor.h>
#include <star/calc/CalGeodeticCoord.h>
#include <star/calc/PostFilter.h>


namespace ss {
namespace calc {

class Interpolation;

class __star_export CCalCoor
{
public:
    CCalCoor();
    ~CCalCoor();

    void setup(const ss::Configure& configure, Interpolation* interpolation);
    void setDeviceType(int deviceType);

    int calcXYZ(SHOTS_CALCOUT_S *currshot) ;           //计算三维坐标

    MultiLevelRegister& multiLevelRegister();
    CalMultiLidarCoor& calMultiLidarCoor();

private:
    //导航设备坐标
    CalMultiLidarCoor m_calMultiLidarCoor;
    MultiLevelRegister m_levelRegister;
    //测绘设备坐标
    CCalibRange m_calibRange;
    CCalibZerosAngle  m_calibZerosAngle;
    CalOneAxisLidarCoor m_calOneAxisLidarCoor ;
    CalTwoAxisLidarCoor m_calTwoAxisLidarCoor ;
    //大地坐标
    CCalGeodeticCoord m_calGeoeticCoor;
    //过滤
    CPreFilter m_preFilter;
    CPostFilter m_postFilter;

    //硬件参数
    int m_deviceType;
};


}
}



#endif //__STAR_SDK_CALC_CAL_COOR_H
