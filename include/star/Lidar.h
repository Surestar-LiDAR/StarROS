/**
 * @author   lucb
 * @date     2020/2/14
 */

#ifndef __STAR_SDK_LIDAR_H
#define __STAR_SDK_LIDAR_H

#include <cstdint>

namespace ss {
namespace lidar {
    enum DeviceModel {
        APilot = 0 ,
        RAngle,
        UArm,
        TPilot,
        AK,
        RFans,
        SL,
        CFans,
        UT = 8 ,
        IRA,
    };

    enum EchoType {     //!<回波类型
        EchoAll = 0,
        EchoOne = 1,    //!< 1回波
        EchoTwo = 2,    //!< 2回波
        EchoTri = 4,    //!< 3回波
        EchoFor = 8,    //!< 4回波
    };

    enum ScanType {     //!<扫描采集类型
        ScanNear = 0,   //!< 近距离模式
        ScanMiddle = 1, //!< 中离模式
        ScanMiddleFar = 2, //!< 中远距离模式
        ScanFar = 3,    //!< 远距离模式
    };


    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
    // *                       测量参数设置
    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
    // 测距方式
    enum RangeMode {
        AMCW = 0xAB,
        TOF  = 0xCB
    } ;

    // 触发沿
    enum TZeorMode {
        Fall = 1 ,         //!< 采集下降沿
        Rise = 0 ,         //!< 采集上升沿
    } ;

    // MPIA_模式
    enum MpiaMode {
        OnlyA = 0,         //!< 仅MPIA_A
        OnlyB = 2,         //!< 仅MPIA_B
        T0Same = 4,        //!< MPIA_A&B，且使用同一T0
        T0Diff = 8,         //!< MPIA_A&B，间隔一个T0
        MFmode = 9
    };

    enum ProgramMode {
        ProgrmModeNon = 0 ,
        ProgrmModeMP = 1 ,
    };
    enum GpsType {        //GPS类型
        GpsApplanix = 0,
        GpsTrimble =1 ,
        GpsNoval = 2 ,
        GpsJavat =3  ,
        GpsNmea = 4,
        GpsGsof = 5,
        GPSInpps = 6,
    };

    enum MirrorNumber {
        oneMirror       = 0,
        twoMirror       = 1,
        threeMirror     = 2,
        fourMirror      = 3,
        zeroAngSignal   = 4,
        updataFrame     = 5,
    };

    enum ScanMode {
        ScanPlusPosA,            //!< 绝对位置模式, 正向旋转
        ScanPlusPosR,            //!< 相对位置模式, 正向旋转
        ScanMinusPosA,           //!< 绝对位置模式, 反向旋转
        ScanMinusPosR,           //!< 相对位置模式, 反向旋转
        ScanPlusSpeed,           //!< 速度模式, 正向旋转
        ScanMinuSpeed            //!< 速度模式, 反向旋转
    };

    // 扫描电机转稳状态
    enum ScanState {
        StandReady  = 0x0 ,  //准备状态
        ScanStand   = 0x1 ,  //转稳状态
        ScanError   = 0x2,
    };

    enum TurretMode {             //!< 转台工作模式
        TurretReset     = 0x00, //!< 重置
        TurretContinue  = 0x01, //!< 连续模式
        TurretArea      = 0x02, //!< 区域模式
        TurretPosition  = 0x04, //!< 位置模式
        TurretNextTrige = 0x08, //!< 下一个曝光点
        TurretStop      = 0x10, //!< 停止
        TurretSearch    = 0x20, //!< 查询
    } ;

    enum ProjectionType {
        UTM     =0,
        GUSS    =1,
    };

    enum EllipsoidType{
        WGS84      =0,
        BeiJing54  =1,
        XiAn80     =2,
        GJ2000     =3,
    } ;

    constexpr static int FREQ_FAR_MIN = 50;
    constexpr static int FREQ_FAR_MAX = 100;
    constexpr static int FREQ_MEDIUM_FAR_MIN = 110;
    constexpr static int FREQ_MEDIUM_FAR_MAX = 200;
    constexpr static int FREQ_MEDIUM_MIN = 210;
    constexpr static int FREQ_MEDIUM_MAX = 450;
    constexpr static int FREQ_NERAR_MIN = 460;
    constexpr static int FREQ_NERAR_MAX = 600;

    constexpr static int DATA_POINT_COUNT =  0x10000 ; //激光点个数为64K

#ifndef T0_STEP
    const static int32_t T0_STEP = 0x80000000; // 2^31, 2147483648
    const static int32_t T0_Clock_Freq = 5000000;	// 5M  //1秒钟的T0 Count值
    const static double T0_Clock_Prcs = 40.0e-6;		// 40PPM
    const static int T0_Clock_Check = (150);  //by zhangwei 将pps的间隔判断值增加到5000150
#endif

    constexpr static int CPT_DELETE_LINE_COUNT = 2 ;  //起始时删除扫描线的个数
    constexpr static int CPT_DELTETE_INDEX = 2;
    constexpr static float CPT_DELTETE_RANGEMIN = 2;
    constexpr static float CPT_DELTETE_RANGEMAX = 3;
    constexpr static int CPT_DELETE_POINT_COUNT = 500 ; //一条扫描线最少点的个数
}
}

#endif //__STAR_SDK_LIDAR_H
