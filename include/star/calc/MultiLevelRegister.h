#ifndef __STAR_SDK_CALC_MULTI_LEVEL_REGISTER
#define __STAR_SDK_CALC_MULTI_LEVEL_REGISTER

#include <star/Star.h>
#include <star/calc/NeighborProcess.h>

#include <string>
#include <star/Configure.h>
#include <vector>
#include <set>

namespace ss {
namespace calc {

class __star_export MultiLevelRegister{
public:
  

    MultiLevelRegister();
    ~MultiLevelRegister() = default;

    void setup(const Configure& configure);

    int setTemperature(float temperature);
    bool set_intensity_params(std::vector<int>& intent,bool isdefault=true);

    //多级数据的处理（默认）
    bool data_level_convert(SHOTS_CALCOUT_S* mtPoint);

    //----------------------0级算法（过滤）-------------------------
    int filterTimeWindow(SHOTS_CALCOUT_S* mtPoint);         // 时间窗口过滤
    int filterRangeIntensityWide(SHOTS_CALCOUT_S* mtPoint); // 距离灰度脉宽过滤
    int filterAngle(SHOTS_CALCOUT_S* mtPoint);              // 角度过滤
    int filterMultiEchoWide(SHOTS_CALCOUT_S* mtPoint);      // 修正多回波对脉宽的影响

    //----------------------1级算法（标定）-------------------------
    //单路标定
	int reviseRange_singeLaser(SHOTS_CALCOUT_S *mtPoint);
	int reviseWide_singeLaser(SHOTS_CALCOUT_S *mtPoint);
	//int reviseIntensity_singeLaser(SHOTS_CALCOUT_S *mtPoint);	

	//修正距离
    int reviseRangeIntensity(SHOTS_CALCOUT_S* mtPoint);  //灰度改正表修正
    int revisePlusMultiCoef(SHOTS_CALCOUT_S* mtPoint);  //加乘系数修正
    int reviseRangeConst(SHOTS_CALCOUT_S* mtPoint);  //加常数修正
    int reviseRangeTemprature(SHOTS_CALCOUT_S* mtPoint);  //修正温度对距离的影响
    int reviseMidFarCalib(SHOTS_CALCOUT_S *mtPoint) ; //中远距离修正

	//拉丝和阳光噪点
	int filterSunNoise_IntensityWide(SHOTS_CALCOUT_S* mtPoint); // 距离灰度脉宽 阳光噪点过滤
	bool filterPoint(size_t no, SHOTS_CALCOUT_S & in_point, const ReviseOptions& reviseOptions);
    //修正脉宽
    int reviseWideRange(SHOTS_CALCOUT_S *mtPoint);   //修正距离对脉宽的影响
    int reviseWideLaser(SHOTS_CALCOUT_S *mtPoint);    //修正激光器间的差异
    //转换为12bit灰度
    int trans12bitIntensity(SHOTS_CALCOUT_S *mtPoint);

    //零位角修正、CFans角度过滤
    int CalbZeroAngle(SHOTS_CALCOUT_S *mtPoint);

    //获取镜面编号
    void findCFansMirrorNum_FPGA(SHOTS_CALCOUT_S *currshot);  //FPGA获取
    void  findCFansMirrorNum_cfans(SHOTS_CALCOUT_S * mtPoint) ;    //上位机获取

    //CFans 边缘角度过滤（角度修正后）
    void findCFansMirrorNum_cfans32(SHOTS_CALCOUT_S *currshot, bool flag);
    void findCFansMirrorNum_cfans128_v1_0(SHOTS_CALCOUT_S *currshot, bool flag);
    void findCFansMirrorNum_cfans128_v2_0(SHOTS_CALCOUT_S *currshot, bool flag);

    //获取角度区间
    void findCFansAngleArea( SHOTS_CALCOUT_S *currshot);
    //CFans安置角度标定
    void reviseAngle(SHOTS_CALCOUT_S *currshot);
	//均匀化处理
	void HomogenizationProcess(SHOTS_CALCOUT_S* currshot);

    //----------------------2级算法（转8bit灰度）-------------------------
    //12bit灰度转换为8bit灰度
    unsigned short trans8bitIntensity(SHOTS_CALCOUT_S *mtPoint);

    //拉丝处理相关算法
    inline bool isNeedFilt() { return need_filt_; }
    inline void setFiltFlag(bool flag = true) { need_filt_ = flag; }
    bool reset();   //重置所有参数

    ReviseOptions& reviseOptions();
    const ReviseOptions& reviseOptions() const;
    void setReviseOptions(const ReviseOptions& reviseOptions);
	void setDefaultReviseOptions( int32_t data_grade);
	void setDefaultReviseOptions_singleCalib( int32_t data_grade);
protected:
   

private:
    std::vector<float> revise_map_;
    float temperature_;
    int intensity_duration_[6]{};
    float params_[10]{};

    std::vector<std::vector<NeighborProcess> >  cloud_procs_vec_;
    bool                                        need_filt_;
    ReviseOptions                               reviseOptions_{};
	
};

}
}

#endif //__STAR_SDK_CALC_MULTI_LEVEL_REGISTER
