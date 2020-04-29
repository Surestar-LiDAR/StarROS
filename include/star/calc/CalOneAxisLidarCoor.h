#ifndef __STAR_SDK_CAL_ONE_AXIS_LIDAR_COOR
#define __STAR_SDK_CAL_ONE_AXIS_LIDAR_COOR

#include <star/Star.h>
#include <star/Configure.h>
#include <star/Shot.h>

namespace ss {
namespace calc {

class __star_export CalOneAxisLidarCoor
{
public:
    CalOneAxisLidarCoor(void);
    ~CalOneAxisLidarCoor(void);
    bool setup(const ss:: Configure& configure);

    int calXYZ(SHOTS_CALCOUT_S *currshot) ;

protected:
    int cal_apXYZ(SHOTS_CALCOUT_S *currshot);
    //  int cal_tpXYZ(SHOTS_CALCOUT_S *currshot);
    int cal_akXYZ(SHOTS_CALCOUT_S *currshot);
    int cal_raXYZ(SHOTS_CALCOUT_S *currshot);
    int cal_utXYZ(SHOTS_CALCOUT_S *currshot);

private:
    int m_deviceType;
    ss::cfg::CommonCalibParams  m_commonCalibParams;
    ss::cfg::ApCalibParams      m_apCalibParams;
    ss::cfg::RaCalibParams      m_raCalibParams;
    ss::cfg::UaCalibParams      m_uaCalibParams;
    ss::cfg::AkCalibParams      m_akCalibParams;
};

}
}

#endif //__STAR_SDK_CAL_ONE_AXIS_LIDAR_COOR
