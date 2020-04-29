
#ifndef __STAR_SDK_CALC_CAL_GEODETIC_COOR_H
#define __STAR_SDK_CALC_CAL_GEODETIC_COOR_H

#include <star/Star.h>
#include <star/Shot.h>

#include <star/Configure.h>
#include <star/calc/math/GeoTrans.h>

namespace ss {
namespace calc {

class Interpolation;

class __star_export CCalGeodeticCoord {
public:
    CCalGeodeticCoord();
    ~CCalGeodeticCoord();

    bool setup(const ss:: Configure& configure,Interpolation *interplt);

    void setDeviceType(int deviceType);

    int calXYZ(SHOTS_CALCOUT_S *currshot) ;

private:
    int intperUTC(SHOTS_CALCOUT_S *currshot) ;
    int calNEDCoor(SHOTS_CALCOUT_S *currshot) ;
    int calGeocentricCoor(SHOTS_CALCOUT_S *currshot);
    int calGeoProjctCoor(SHOTS_CALCOUT_S *currshot) ;


    PosMsg_S m_pos;
    //double m_anlignAngle[32][3];

	

    Interpolation *m_inter;
    ///PRE_IMU_S m_ldrPos ;
    //PRE_GeoProject_S m_geocvt;
    // POSModuleSelect_S  m_posSelect;


    ss::cfg::Interpolation m_interplation;
    ss::cfg::PosCmpCoor m_posCmpCoor;
    ss::cfg::GeoProjection m_geoProjection;
    math::CGeoTrans m_geoTrans;

	ss::cfg::AnlignAngleRFans  m_anlignAngle;

    int m_deviceType;
};

}
}


#endif //__STAR_SDK_CALC_CAL_GEODETIC_COOR_H
