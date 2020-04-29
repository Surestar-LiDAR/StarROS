#ifndef __STAR_SDK_CALC_PRE_FILTER
#define __STAR_SDK_CALC_PRE_FILTER

#include <star/Star.h>

#include <star/Shot.h>
#include <star/Configure.h>

#include <star/Configure.h>

namespace ss {
namespace calc {

//typedef struct _fa_S {  //角度滤波选项：过滤范围内的数据
//    bool angleInvert;                        //by zhubing 2018.7.20
//    double angleMin;
//    double angleMax;
//    double angleMin_filterAll;
//    double angleMax_filterAll;
//    double angleMin_filter[5];
//    double angleMax_filter[5];
//    double FreqMin_filter[3];
//    double FreqMax_filter[3];
//    double rangeMin_filter[3][3];
//    double rangeMax_filter[3][3];
//    double intensityMin_filter[3][3];
//    double intensityMax_filter[3][3];
//    _fa_S () {
//        angleInvert = 0;               //by zhubing 2018.7.20
//        angleMin = 0.0 ;
//        angleMax = 360.0;
//    }
//} PRE_FltAngl_S ;

//typedef struct _fr_S {     //距离、强度滤波选项:  过滤范围内的数据
//    float rngMin , rngMax ;  //!< 测距最小最大值
//    int intMin, intMax ;     //!< 强度最小最大值
//    _fr_S () {
//        rngMin = 1.0 ;
//        rngMax = 3000.0 ;
//        intMin = 0 ;
//        intMax = 2047 ;
//    }
//} PRE_FltRInt_S ;

//typedef struct RangeIntFilt{
//    double R0;
//    double  R1;
//    double  R2;
//    double  R3 ;
//    double I0;
//    double  I1;
//    double  I2;
//    double  I3;
//    double  I4;
//    double  I5;
//    unsigned int PointFreqMin ;
//    unsigned int   PointFreqMax;
//    RangeIntFilt() {
//        R0 = R1 = R2 =0 ;  R3 = 10000 ;
//        I0 =  I1 = I2 = I3 = I4 = 0;  I5 = 8192;
//        PointFreqMin=0;  PointFreqMax=600 ;
//    }
//}RANGE_ITNENSITY_FILT ;

class CPreFilter {
public:
    CPreFilter(void);

    ~CPreFilter(void);

    bool setup(const ss::Configure& configure);

    int filterProcess(SHOTS_CALCOUT_S* currshot);

private:
    // bool filtRI(double range ,unsigned short intensity) ;
    int filterResample(SHOTS_CALCOUT_S* currshot);

    int filterAngle(SHOTS_CALCOUT_S* currshot);

    //PRE_FltAngl_S m_fltAngle ;
    //PRE_FltRInt_S m_filpara;
    //unsigned int m_PointFreq ;
    //int m_ratio;
    // RANGE_ITNENSITY_FILT m_RangeRangeFilt, m_RangeRangeFilt1 ,m_RangeRangeFilt2 ,m_RangeRangeFilt3 ;
    // bool m_resampleModule, m_AngleModule ,m_rangeModule,m_RangeIntModule ,m_AnglePointFreqModule ,m_RangeByIntensityModule ;
    // PRE_TR_S m_validR;
    // FilterModuleSelect_S m_filterModule ;

    ss::cfg::AngleFilter m_angleFilter;
    ss::cfg::Resample m_resample;
    int m_pointNum;
};

}
}

#endif //__STAR_SDK_CALC_PRE_FILTER
