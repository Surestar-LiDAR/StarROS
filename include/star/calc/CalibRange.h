#ifndef __STAR_SDK_CALC_CALIB_RANGE_H
#define __STAR_SDK_CALC_CALIB_RANGE_H

#include <star/Star.h>
#include <star/Lidar.h>
#include <star/Configure.h>
#include <star/Shot.h>

namespace ss {
namespace calc {

class __star_export CCalibRange {
public:
    CCalibRange();
    ~CCalibRange();

    bool setup(const ss::Configure& configure);
    int calibRange(SHOTS_CALCOUT_S* currshot);

protected:
    int calibRangeIntensity(SHOTS_CALCOUT_S* currshot);
    int calibRangeConstant(SHOTS_CALCOUT_S* currshot);
    bool calibRangeByTemper(SHOTS_CALCOUT_S* currshot, double temperature);

private:
    float m_rangecorrA[cfg::INTENSE_MAX];
    float m_rangecorrB[cfg::INTENSE_MAX];

    double p1_A;
    double p2_A;
    double p1_B;
    double p2_B;
    double p1;
    double p2;

    int m_scdVersion;
    lidar::ScanMode m_scanMode;
    lidar::DeviceModel m_deviceType;

    cfg::RangeTemperPara m_rangeTemperPara;
    cfg::RangeConstPara m_rangeConstPara;
    cfg::IntensityTable m_intensityTable;
};

}
}

#endif //__STAR_SDK_CALC_CALIB_RANGE_H
