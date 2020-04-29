#ifndef __STAR_SDK_CALC_CAL_MULTI_LIDAR_COOR_H
#define __STAR_SDK_CALC_CAL_MULTI_LIDAR_COOR_H

#include <star/Star.h>

#include <star/Configure.h>
#include <star/Shot.h>
#include <star/calc/MultiLevelRegister.h>

namespace ss {
namespace calc {

class __star_export CalMultiLidarCoor
{
public:
    CalMultiLidarCoor(void);
    ~CalMultiLidarCoor(void);
    bool setup(const ss::Configure& configure) ;
    int  calcXYZ(SHOTS_CALCOUT_S *currshot,const ReviseOptions& reviseOptions);

    MultiLevelRegister& multiLevelRegister();
    const MultiLevelRegister& multiLevelRegister() const;

protected:
    ////1.初始化角度参数
    //int init_CFans32();
    //int init_CFans128();
    //int init_CFans128_v2_0();
    // int initPara(unsigned short dataid) ;

    //2.解算RFans的三维坐标
    int calcRFansXYZ(SHOTS_CALCOUT_S *currshot);
    double calcTheta(unsigned short dataid, unsigned short laserid);

    //3.解算CFans的三维坐标
    void calcCFansXYZ(SHOTS_CALCOUT_S *currshot,const ReviseOptions& reviseOptions);
	void calcCFans(SHOTS_CALCOUT_S* currshot, double (*angleCorr)[12]);
    void calcCFansXYZ_32(SHOTS_CALCOUT_S *currshot);
    void calcCFansXYZ_128_V2(SHOTS_CALCOUT_S *currshot);
    void calcCFansXYZ_128_V1(SHOTS_CALCOUT_S *currshot);

    //Cfans_Angle_Range m_CfansAngleRange_32, m_CfansAngleRange_128, m_CfansAngleRange_128_v20;
    //Cfans_Angle_Range *m_CfansAngleRange;
    //CALIB_SHARE_S m_calibpara;

private:
    int m_deviceType;
    MultiLevelRegister  m_levelRegister;
    ss::cfg::CommonCalibParams  m_commonCalibParams;
	ss::cfg::LaserRangeConstPara m_laserRangeConstPara;
	//bool m_useCFansFPGA;
};

}
}

#endif //__STAR_SDK_CALC_CAL_MULTI_LIDAR_COOR_H
