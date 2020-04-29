#ifndef __STAR_SDK_CALC_CALIB_ZEROS_ANGLE_H
#define __STAR_SDK_CALC_CALIB_ZEROS_ANGLE_H

#include <star/Star.h>
#include <star/Lidar.h>
#include <star/Shot.h>
#include <star/Configure.h>

namespace ss {
namespace calc {

class __star_export CCalibZerosAngle
{
public:
    CCalibZerosAngle(void);
    ~CCalibZerosAngle(void);

    bool setup(const ss::Configure& configure);
    int calibAngle(SHOTS_CALCOUT_S* currshot);
protected:
    int AP_calibAngle(double *scanner,lidar::MirrorNumber *mirrorNumb);
    int TP_calibAngle(double *scanner,lidar::MirrorNumber *mirrorNumb);
    int RA_calibAngle(double *scanner);
    int UA_calibAngle(double *scanner,double * turnAngle);
    int AK_calibAngle(double *scanner);
    int SL_calibAngle(double *scanner,double * turnAngle) ;
    int UT_calibAngle(double *scanner,lidar::MirrorNumber *mirrorNumb) ;

private:
    int  m_deviceType;
    ss::cfg::CommonCalibParams  m_commonCalibParams;
    ss::cfg::ApCalibParams      m_apCalibParams;
    ss::cfg::RaCalibParams      m_raCalibParams;
    ss::cfg::UaCalibParams      m_uaCalibParams;
    ss::cfg::AkCalibParams      m_akCalibParams;

    bool isSeparate;
    bool isOneMirror;
    bool isTwoMirror;
    bool isThrMirror;
    bool isFourMirror;
    int m_ViewAngle;
};

}
}



#endif //__STAR_SDK_CALC_CALIB_ZEROS_ANGLE_H
