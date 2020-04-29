#ifndef __STAR_SDK_CALC_POST_FILTER
#define __STAR_SDK_CALC_POST_FILTER

#include <star/Star.h>

#include <star/Shot.h>
#include <star/Configure.h>

namespace ss {
namespace calc {

//typedef struct _FilHeightPara{
//    double minHeight;
//    double maxHeight;
//    _FilHeightPara(){
//        minHeight = 0;
//        maxHeight = 3500;
//    }
//}FilHeightPara_S;

class __star_export CPostFilter
{
public:
    CPostFilter(void);
    ~CPostFilter(void);

	bool setup(const ss:: Configure& configure);
    int filterProcess(SHOTS_CALCOUT_S *currshot);

private:
	//FilHeightPara_S m_heightpara;
	//FilterModuleSelect_S m_filterModule ;

	ss::cfg::HeightFilter  m_heightFilter;
	ss::cfg::RangeFilter m_rangeFilter;
	ss::cfg::RangeIntensityFilter m_rangeIntensityFilter;
	ss::cfg::RangeByIntensityFilter     m_rangeByIntensityFilter;
	ss::cfg::PointFreqAngleRangeFilter m_pointFreqAngleRangeFilter;

	int m_PointFreq;
	int filterHeight( SHOTS_CALCOUT_S *currshot );
	int filterRange(SHOTS_CALCOUT_S *currshot) ;
    int filterRangeIntensity(SHOTS_CALCOUT_S *currshot) ;
    int filterAngle_bypointFreq(SHOTS_CALCOUT_S *currshot);
    int filterRangeByIntensity(SHOTS_CALCOUT_S *currshot);

	bool filtRI(double range, unsigned short intensity, int index);
};

}
}

#endif //__STAR_SDK_CALC_POST_FILTER