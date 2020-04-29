#ifndef __STAR_SDK_CALC_MATH_CONVERT_H
#define __STAR_SDK_CALC_MATH_CONVERT_H

#include <vector>
#include <time.h>

#include <star/Star.h>
#include <star/Lidar.h>

namespace ss {
namespace calc {
namespace math {

void TranToWGS84(int CentralMeridian,double *pos);
void TranToCoorXYZ(double a,double b,double *XYZ,double *pos);

double deg2rad(double arg); //!< 角度转换为弧度
double rad2deg(double arg); //!< 弧度转换为角度
void BL_GUSS(double sa,double sb,double l,double b,double *x,double *y) ;

}
}
}

#endif // __STAR_SDK_CALC_MATH_CONVERT_H
