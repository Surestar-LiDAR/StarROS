#ifndef __STAR_SDK_CALC_MATH_GEO_TRANS_H
#define __STAR_SDK_CALC_MATH_GEO_TRANS_H

#include <star/Star.h>

namespace ss {
namespace calc {
namespace math {

class __star_export CGeoTrans {
public:
    CGeoTrans(void);

    ~CGeoTrans(void);

    double m_a, m_f;

    // [纵B 横L 中央经L0 纵常量(dx=0) 横常量(dy=500000) 纵直角X 横直角Y]
    void BLToXY(double B, double L, double CentL, double dx, double dy, double& x, double& y);

    // [纵直角X 横直角Y 中央经L0 纵常量(dx=0) 横常量(dy=500000) 纵B 横L ]
    void XYToBL(double X, double Y, double CentL, double dx, double dy, double& B, double& L);

    double dms2rad(double x);

    double rad2dms(double x);

    bool MatrixInversion_General(double* a, int n);

    bool MatSAT(double* a, int m);

    bool MatSAT(double* a, double* at, int m, int n);

    void Zero(double* p, int n);

    bool MatrixMulti(double* a, double* b, double* c, int m, int n, int l);

    bool MatrixSub(double* a, double* b, double* c, int m, int n);

    //求指定两正交矢量作为X/Z轴坐标系与原坐标系的旋转矩阵R
    bool CalRM(double* R, double* X, double* Z);

    //(B L)==>(X Y Z)
    void CalEarthCentXY(double* XYZ, double B, double L);

    //已知P点 平面上的一点O与平面单位法向量，求P在平面的垂足坐标
    void CalVPF(double* XYZ, double* P, double* O, double* N);

    //惯导摄影测量系统的6个旋转计算
    //惯导自身坐标系b；像辅惯导坐标系bp;导航坐标系n；地心坐标系e；高斯坐标系np；
    //大地坐标系E；像空间坐标系B;
    //double Tbpb[9],CBbp[9],CEB[9],TnpE[9],Tnnp[9],Cbn[9];
    void CalTMat(double* Tbpb, double* TnpE);

    void CalIMU(double* R, double roll, double pitch, double yaw);

    void CalCMR(double* R, double omg, double phi, double kap);

    void CalGassMat(double* R, double Lgk, double B0, double L0, double B, double L);

    void CalErrMat(double* R, double ex, double ey, double ez);

    void CEB_ANGLE(double* R, double& omg, double& phi, double& kap);

    void Cnb_ANGLE(double* R, double& rol, double& pit, double& yaw);

    void NormalAngle(double& p, double& o, double& k);

    double NRAngle(double rad);

    void RRx(double* R, double angle);

    void RRy(double* R, double angle);

    void RRz(double* R, double angle);

    //坐标系统设置 统一单位为（米 度） X横坐标输入可以加带号，但输出去掉了代号
    void SetCoorSysType(int ty = 84, bool inip = true);

    void SetL0(double L0, double HX = 500000, double ZX = 0); //中央子午线及平移参数设置
    double GetL0(double LL, int ty = 3);

    void LB2XY(double L, double B, double& X, double& Y);//统一为横坐标 纵坐标
    void XY2LB(double X, double Y, double& L, double& B); //
    void Z54D84(double L0, double X54, double Y54, double H54, double& L, double& B, double& H);

    void Z80D84(double L0, double X80, double Y80, double H80, double& L, double& B, double& H);

    void L54D84(double L0, double L54, double B54, double H54, double& L, double& B, double& H);

    void L80D84(double L0, double L80, double B80, double H80, double& L, double& B, double& H);

    void LB84XY80(double L0, double L, double B, double H84, double& X, double& Y, double& H);

    void Xyz2Lbh(double X, double Y, double Z, double& L, double& B, double& H);

    void Lbh2Xyz(double L, double B, double H, double& X, double& Y, double& Z);

    double rangle(double L, double B); //计算午线收敛角

protected:
    void CalEP();//计算对应椭球内部参数

private:
    double mL0, m_dZX, m_dHX; //中央子午线（弧度 纵向平移ZX=0 横向平移HX=500000)
    double mRC, EE;// 椭球第二偏心率
    double CC1, CC2, CC3, CC4; //椭球中间参数
    double d2r, r2d; //度转弧度系数 和 弧度转度系数
    double TT, TE, TC;//转换常数

    double DX80_84, DY80_84, DZ80_84, DX54_84, DY54_84, DZ54_84;
};

}
}
}


#endif //__STAR_SDK_CALC_MATH_GEO_TRANS_H
