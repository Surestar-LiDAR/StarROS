/**
 * @author   lucb
 * @date     2019/12/12
 */

#ifndef __STAR_SDK_IMP_HEADER_H
#define __STAR_SDK_IMP_HEADER_H

#include <star/msg/Message.h>
#include <star/msg/detail/basic_message.h>

namespace ss {
namespace msg {
namespace scd {
namespace v1 {

constexpr static std::size_t IMP_SYNCWORD_COUNT = 8;

#ifndef PROJ_STR_SIZE
constexpr static std::size_t PROJ_STR_SIZE = 31;
constexpr static std::size_t SCADA_VER_LEN = 32;
constexpr static std::size_t CMR_NUMBER = 6;
constexpr static std::size_t ROI_NUMBER = 2;
#endif

constexpr static std::size_t INTENSE_MAX = 2048;
constexpr static std::size_t SCDSCAN_TYPE_COUNT = 3;
constexpr static std::size_t USBCMR_MAX_NUMBER = 2;

#pragma pack(push, 1)
//服务器的系统时间，用于在开始时执行系统时间同步。
typedef struct {
    uint16_t wYear;
    uint16_t wMonth;
    uint16_t wDayOfWeek;
    uint16_t wDay;
    uint16_t wHour;
    uint16_t wMinute;
    uint16_t wSecond;
    uint16_t wMilliseconds;
} CSYSTEMTIME;

typedef struct {
    int8_t syncword[IMP_SYNCWORD_COUNT]; //0xA7A7A7A7
    uint32_t formatVer;
    uint32_t timeStamp;           //数据采集同步时间对
    uint16_t stationNbr;
    char deviceID[32];
    char scadaAddr[32];          // scada IP address
    int32_t deviceType;
    uint32_t deviceRev;     //!< 设备编号
    uint32_t productDate;   //!< 升级信息  - from SCDCFG_VERSION_S
    // LiDAR_PRIORITY_S
    char projectID[PROJ_STR_SIZE + 1];
    char customID[PROJ_STR_SIZE + 1];
    char license[PROJ_STR_SIZE + 1];
    uint16_t workTime;       // 工作总时间
    uint16_t laserTime;      // 激光器工作总时间
    uint32_t fpgaVer;          //FPGA 版本信息
    char scadaVer[SCADA_VER_LEN];  //SCADA 版本信息
    CSYSTEMTIME sysTime;
    uint32_t systemState;      //系统状态
    uint32_t impVer;              //IMP 版本信息
// added by zhubing 2020.2.15
    uint32_t m_cycle;              //光脉冲数
    int8_t reserver[766 - 4];             //total 1020Byte
} SCADA_META_S; //-- IMP_LidarHeader_S ;

//! 激光器配置
typedef struct {
    int32_t lsrType;       //!< 激光器类型
    int32_t freqMin;                //!< 激光器最小频率 单位:KHz
    int32_t freqMax;                //!< 激光器最大频率 单位:KHz
    int32_t elecMin;                //!< 激光器最小电流 单位:mw
    int32_t elecMax;                //!< 激光器最大电流 单位:mw
    int32_t powMin;                //!< 用户配置激光最小功率
    int32_t powMax;                //!< 用户配置激光最大功率
    int32_t rangeMin;              //!< 激光器最小距离
    int32_t rangeMax;              //!< 激光器最大距离
    int32_t workPowMin;            //!< 激光工作最小功率
    int32_t workPowMax;            //!< 激光工作最大功率
    int32_t reserved[5];            //!< 保留  //total 64
} SCDCFG_LASER_S;

//! 电机高级配置
typedef struct {
    int32_t motType;                //!< 电机类型
    float spdMin;                 //!< 电机最小转速
    float spdMax;                 //!< 电机最大转速
    int32_t coder;                  //!< 电机编码器
    float zeroAng;              //!< 电机零位角 or 转台分辨率
    // 视场起始角度 - 视场终止角
    uint32_t angstr[4];
    uint32_t angstp[4];
    uint16_t scnSpdCheck;
    uint16_t stgType;    //0: bcd ; 1: acj
    uint32_t periodOrPichHigh;    //  or PITCH_all
    int8_t reserved[4];  //total 64
} SCDCFG_MOTOR_S, SCDCFG_STAGE_S;

//! 相机高级配置
typedef struct {
    int32_t cmrType;                //!< 相机曝光类型
    int32_t cmrMode;                //!< 相机记录模式
    int32_t plsWidth;               //!< 相机电平脉宽
    int32_t cmrCount;               //!< 相机个数
    float circum;               //!< 车轮周长
    int32_t dmiEncoder;            //!< DMI 编码器线数
    uint32_t shutOffset[CMR_NUMBER];   //CMR_NUMBER个相机曝光延时寄存器
    int32_t cmrFlashSign;          //1：USB相机外触发 0：USB相机内触发
    int32_t reserved[3];            //!< 保留  //total 64
} SCDCFG_CAMERA_S;

typedef struct {
    int32_t nearPowMin_;   //!<
    int32_t nearPowMax_;   //!<
    int32_t farPowMin_;   //!<
    int32_t farPowMax_;   //!<
    int32_t nearRangeMin_;   //!<
    int32_t nearRangeMax_;   //!<
    int32_t farRangeMin_;   //!<
    int32_t farRangeMax_;   //!<
    int32_t reserved[8];    //!<   64 Byte
} SCDCFG_RANGE_S;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                       测量参数设置
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
typedef struct {
    int32_t rangeMode;          //!< 测距模式
    int32_t mpiaMode;           //!< 工作模式
    int32_t tzeroMode;          //!< 测量信号沿
    float pulseWdthA_;         //!< GPXA 脉冲宽度 liyp  过时
    float pulseWdthB_;         //!< GPXB 脉冲宽度 liyp  过时
    int32_t peakSync;           //!< 高倍峰值同步
    int32_t peakSyncL;           //!< 低倍峰值同步L
    int32_t simuRatio;          //!< 仿真数据分配
    int32_t tdcaFact;           //!< TDC_A精度系数
    int32_t tdcbFact;           //!< TDC_B精度系数
    int32_t mSet;                //!< 单回波精度系数
    float af;                   //!< 指南针磁偏角  liyp /*电子水泡精度系数*/
    uint32_t packEnable;         //!< 数据打包使能
    uint16_t selecPeak;
    uint16_t scdVersion;
    uint16_t timeDelay;           //!< 时间窗口延时
    uint16_t pulseCtrl;
    uint16_t pulseDelay;
    uint16_t selecPeakL;
    // int16_t reserved[1];       //total 64 Byte
} SCDCFG_RULER_S;

typedef struct {
    int32_t tempType;           //!< 温度传感器类型
    int32_t tempMin;            //!< 低温默认值
    int32_t tempMax;            //!< 高温默认值
    int32_t gpsType;     //!< GPS类型默认值：eGpsTrimble
    uint16_t imuTypeSelect; //0：uIMU-IC;  1:LCI-100  2:HG4930CA51
    int16_t reserved[23];       //total 64 Byte
} SCDCFG_ENVIR_S;

typedef struct {
    float areaStrAngMin, areaStrAngMax;
    float areaStpAngMin, areaStpAngMax;
    float areaItvAngMin, areaItvAngMax;
    float angTrgStrMin, angTrgStrMax;
    float angTrgStpMin, angTrgStpMax;
    float angTrgItvMin, angTrgItvMax;
    float gridHmin, gridHmax;
    float gridVmin, gridVmax;
    int32_t gridrefRangeMin, gridrefRangeMax;
    float radianHmin, radianHmax;
    float radianVmin, radianVmax;
    float roiHstrAngMin, roiHstrAngMax;
    float roiHstpAngMin, roiHstpAngMax;
    float roiVstrAngMin, roiVstrAngMax;
    float roiVstpAngMin, roiVstpAngMax;
    int32_t cmrTimeItvMin, cmrTimeItvMax;
    int32_t cmrRangeItvMin, cmrRangeItvMax;
    //int8_t reserved[120] ; //total 256
    int32_t reserved[30];
} SCDCFG_CHECK_S;

constexpr uint16_t LSR_SEL_NUM = 16;
typedef struct {
    uint16_t laserSel[LSR_SEL_NUM];
} SCDCFG_LSRSEL_S;

constexpr uint16_t FREQ_NUM = 7;
typedef struct {
    uint16_t freq[FREQ_NUM];
    uint16_t nflight;//n倍光程
} SCDCFG_FREQS_S;

constexpr uint16_t ANGEL_NUM = 16;
typedef struct {
    uint16_t angle[ANGEL_NUM];
} SCDCFG_ANGELS_S;

typedef struct {
    uint32_t lsrRevSel_0_7;
    uint32_t lsrRevSel_8_15;
    uint32_t lsrRevTiem_0_3;
    uint32_t lsrRevTiem_4_7;
    uint32_t lsrRevTiem_8_11;
    uint32_t lsrRevTiem_12_15;
    int32_t reserved[10];//total 64 Byte
} SCDCFG_RLSR_CNL_S;

typedef struct {
    float range[16];
} SCDCFG_RLSR_RANGE_S;

//! 系统配置参数

typedef struct {
    //zks    SCDCFG_VERSION_S version;           //!< 版本信息
    int32_t escadaType;
    SCDCFG_LASER_S lsrConf;      //!< 激光器高级配置
    SCDCFG_MOTOR_S scnConf;      //!< 电机高级配置
    SCDCFG_STAGE_S stgConf;      //!< 转台配置  -- zks
    SCDCFG_CAMERA_S cmrConf;      //!< 相机高级配置
    SCDCFG_RULER_S rlrConf;      //!< 测量精度配置
    SCDCFG_ENVIR_S envConf;      //!< 环境控制配置
    SCDCFG_RANGE_S lrsRangCfg_;  //!< 近远距离边界配置
    SCDCFG_CHECK_S bdChkConf_;   //!< 设备能力
    SCDCFG_LSRSEL_S lsrSelConf;  //!<不同扫描区间激光频率配置
    SCDCFG_FREQS_S freqsConf;     //!<可选激光频率配置
    SCDCFG_ANGELS_S anglesConf;   //!扫描电机视场角或激光变频区间
    SCDCFG_RLSR_CNL_S rfansLsr;
    SCDCFG_RLSR_RANGE_S rRangeConf;
    int8_t reserved[1132];           //!< total 2KB
} SCADA_CONFIG_S;

//Can only read from config file, not from the cmd line or GUI
typedef struct {
    float p1_A_far;      //50k-100k对应检校参数
    float p2_A_far;
    float p1_B_far;
    float p2_B_far;

    float riArrayA[INTENSE_MAX];  // gpxA灰度范围[0-2047]     //50k
    float riArrayB[INTENSE_MAX]; // gpxB灰度范围[0-2047]
    float riArrayC[INTENSE_MAX];  // gpxA灰度范围[0-2047]     //300k
    float riArrayD[INTENSE_MAX]; // gpxB灰度范围[0-2047]
    float riArrayE[INTENSE_MAX];  // gpxA灰度范围[0-2047]     //500k
    float riArray9[INTENSE_MAX]; // gpxB灰度范围[0-2047]

    float p1_A_medium_far;  //110k-200k对应检校参数
    float p2_A_medium_far;
    float p1_B_medium_far;
    float p2_B_medium_far;

    float p1_A_medium;    //210k-450k对应检校参数
    float p2_A_medium;
    float p1_B_medium;
    float p2_B_medium;
    float p1_A_near;      //460-600k对应检校参数
    float p2_A_near;
    float p1_B_near;
    float p2_B_near;
    //int8_t reservedB[16320] ;

    float temperatureStart;
    float temperatureEnd;
    float temperatureCoff2;
    float temperatureCoff1;
    float temperatureCoff0;
    int8_t reservedB[16300];

////距离小于10的距离检校参数
    //float p1_A_far_10;      //50k对应检校参数
    //float p2_A_far_10;
    //float p1_B_far_10;
    //float p2_B_far_10;
    //float p1_A_medium_10;    //300k对应检校参数
    //float p2_A_medium_10;
    //float p1_B_medium_10;
    //float p2_B_medium_10;

//  float p1_A_near_10;      //500k对应检校参数
//  float p2_A_near_10;
//  float p1_B_near_10;
//  float p2_B_near_10;
//  int8_t reservedB[16288] ;
//  int8_t reservedB[132080] ;         //!< 193KB
} RANGE_CALIB_S;

typedef struct {//zks:为什么用字符串？
    char srCalibs[256]; //参数个数长度最大为256
    char calibs[256];
    char urInclt[256];
    char RIfilt[256];
    char reserved[1024];  //total 2KB
} SCDCAL_ARRAY_S;

//module parameters
typedef struct {
    //int8_t para[17][128];   //!< 每个section的算法模块最大17.
    //int8_t reserved[1920] ; //!< total 4KB
    char para[25][128];   //!< 每个section的算法模块最大17.
    char reserved[896]; //!< total 4KB
} ALGSC_ARRAY_S;

//! scanner
typedef struct {
    int32_t cmdstat;
    int32_t mode;
    float angSpeed;           //!< depends on SCDSCN_MODE_E
    uint16_t roistr[4];
    uint16_t roistp[4];
    uint32_t speedCount;   //扫描电机count
    int32_t speedState;  //扫描电机转速状态
    int32_t grade;
    int16_t reserved[10];  //total 60 Byte
} SCDCMD_SCANER_S;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                            转  台
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
//! turret
typedef struct {
    int32_t cmdstat;
    int32_t mode;
    float angSpeed;
    float strAngle;
	float stpAngle;          //!< 区域起始角度
    float currAng;                     //!< 当前角度
    uint16_t roistr[4];
    uint16_t roistp[4];
    int32_t currMiStat;
    int32_t currMaStat;
    int32_t reserved[3];  //total 60 Byte
} SCDCMD_TURRET_S;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                            步进电机
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
typedef struct {
    int32_t cmdstat;
    int32_t mode;//旋转方向
    float angle;//转动角度
    float currAng;//当前角度 绝对位置编码器当前角度
    int32_t reserved[11]; //total 60 Byte
} SCDCMD_STEPMOTOR_S;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                            环   境
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
//! 设备环境配置
typedef struct {
    int32_t gpsType;      //!< GPS类型默认值：eGpsTrimble
    uint32_t utcTime;    //!< 年月日时
    uint32_t utcSec;     //!< UTC周秒
    int8_t pps;               //!< PPS信号 0 :无信号   非零：有信号
    uint8_t satelliteCount;    //!< gps 卫星个数
    uint8_t imuAmSign;         //!< IMU 数据异常报警信号 0: 无报警信号  非零：有报警
    uint8_t imuDataState;     //!< IMU 数据传输状态 0: 数据停止传输  非零：数据传输
    float fGPSx;             //!< 纬度
    float fGPSy;             //!< 经度
    float fGPSz;             //!< 海拔
    float temp[2];            //!< 温度
    float hemi[2];            //!< 湿度
    int16_t altitude;          //!< 高度
    uint16_t airspeed; //!< 空速
    float incX;              //!< inclinometer X
    float incY;              //!< inclinometer Y
    float compassAng;         //!< 指南针角度
} SCDCMD_ENVIR_S;

typedef struct {
    int32_t cmdStat;        //!< 相机控制
    int32_t triggerMode;  //!< 曝光模式
    int32_t interval;               //!< 时间/角度 间隔
    int32_t angOffset;              //!< 角度偏移
    uint32_t msgCount;       //!< 命令编号
    uint32_t flashCount[CMR_NUMBER];
    uint32_t trigCount;

    float strAngle;
	float stpAngle;   //!< 区域起始角度
    int32_t reserved[1];            // total 60 Byte
} SCDCMD_CAMERA_S;

typedef struct {
    int32_t index;
    int32_t brightness;
    int32_t contrast;
    int32_t hue;
    int32_t saturation;
    int32_t sharpness;
    int32_t gamma;
    int32_t wbalance;
    int8_t trigger;
} USBCMR_CTRL_S;

typedef struct {
    int32_t cmdStat;         //!< USB相机控制
    int32_t triggerMode;   //!< 曝光模式
    float currAngle;            //!< 当前转台trigger角度
    float itvAngle;              //!< 曝光间隔角度
    float strAngle;
	float stpAngle;    //!< 区域起始角度
    int8_t trigger[USBCMR_MAX_NUMBER];
    float expValues[USBCMR_MAX_NUMBER]; //曝光时间
    uint16_t grayValue[USBCMR_MAX_NUMBER]; //灰度值
    int16_t reserved[11];    //total 60 Byte
} USBCMR_TRIG_S;

typedef struct {
    int8_t pps;           //!< PPS信号
    int8_t utc;           //!< UTC信号
    int8_t gps;           //!< GPS信号
    int8_t reservedA[1]; //total 60 Byte
    uint32_t tZero;
    CSYSTEMTIME syncTime;
    int8_t reserved[36]; //total 60 Byte
} SCDCMD_TIMSYN_S;

//palm 数据传输参数
typedef struct {
    int32_t cmdStat;         //数据传输使能
    int32_t imuState;        //imu数据打包
    int32_t gpsState;        //gps数据打包
    uint32_t dmiMileage;    //dmi 里程数 单位m
    int32_t gpsType;       //GPS类型
    uint32_t utcTime;    //UTC时间秒
    uint32_t utcYMDH;   //UTC年月日时
    uint16_t stateCount;  //卫星数
    uint16_t ppsSign;   //!< PPS信号  0 :无信号   1：有信号
    uint16_t imuUtcSync;    //imu UTC时间同步 0 :未同步   1：同步
    uint16_t imuDataState;  //imu 数据传输状态 0: 数据停止传输  非零：数据传输
    uint16_t gpsDataSate;  //gps 数据传输状态 0: 数据停止传输  非零：数据传输
    uint16_t switchStat;    //按键状态
    uint32_t issDataSize;
    int8_t reserved[16]; //total 60 Byte
} SCDCMD_PALM_S;

//palm 转台
typedef struct {
    int32_t cmdStat;        //转台使能
    int32_t mode;     //转台工作模式
    float speed;                //角度(度)/速度(dps)
    float angle;                //转动角度
    int32_t dire;     //方向
    float pulseAng;             //每个脉冲对应的角度
    int32_t cmrStat;        //相机使能
    float crtAngle;               //当前转动角度
    int8_t reserved[28];          //!<total 60 Byte
} SCDCMD_PALMSTG_S;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                           数据采集
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

typedef struct {             //同时设置数据采集模式0
    int32_t cmdstat;
    int32_t m_eEcho;      //!< 回波模式
    int32_t rangemin;
	int32_t rangeMax;   //!< meters
    int32_t mpMode;
    uint16_t isNetFlow; //storageSetup()->setup 1控制设备（PC，PAD）存储；2: 设备存储；8: U盘存储
    int16_t roiSstr[ROI_NUMBER];
    int16_t roiSstp[ROI_NUMBER];
    int16_t roiTstr[ROI_NUMBER];
    int16_t roiTstp[ROI_NUMBER];
    uint16_t itvWorkTime;   //设备间隔工作时间 单位秒
    int8_t reserved[6];             //!<total 60 Byte
    int32_t fileState;
    int32_t dataType;       //storageSetup()->setup
    int32_t scanMode;    //近中远模式
    uint16_t resetSign;    //重置RW 脉冲计数
} SCDCMD_PROGRM_S;

//! 寄存器访问
typedef struct {
    uint32_t regAddr;          //!< 寄存器地址
    uint32_t regValue;         //!< 寄存器数据
    int32_t reserved[13];             //!<total 60 Byte
} SCDCMD_REGIST_S;

typedef struct {
    int32_t state;
    uint16_t vol;
    uint16_t amp;       //!< 工作电压，电流
    uint16_t humidA;         //!< 湿度A
    uint16_t tempA;           //!< 温度A
    uint16_t humidB;          //!< 湿度B
    uint16_t tempB;           //!< 温度B
    uint16_t altitude;       //!< 工作高度
    uint32_t tdcaFact;        //!< 精度系数A  FPGA 寄存器返回  liyp
    uint32_t tdcbFact;        //!< 精度系数B  FPGA 寄存器返回  liyp
    uint16_t mSet;           //!< 单回波精度系数  FPGA 寄存器返回 liyp
    int8_t reserved[32];             //!<total 60 Byte
} SCADA_STATE_S;

//!< FPGA 状态监控
typedef struct {
    uint8_t hardState;    //!< T0脉冲
    uint8_t packState;  //!< 数据打包
    uint8_t tranState;  //!< 数据传输
    uint8_t simuState;  //!< 仿真测试模式
    int32_t reserve[14];  //total 60 Byte
} SCADA_FPGA_STATE;

typedef struct {
    int32_t cmdstat;
    int32_t storePlace; //数据存储位置
    int32_t storeType;          //数据存储类型
    int32_t reserve[12];  //totao 60 Byte
} SCADA_DATAFUN_S;

typedef struct {
    int32_t cmdstat;
    int32_t dmiEncoder;             //!< DMI 编码器线数
    float dmiCircum;            //!< DMI 周长 m
    int32_t dmiSelect;
    int32_t reserve[11];  //totao 60 Byte
} SCDCMD_DMI_S;

typedef struct {
    int32_t cmdstat;     //!< 激光器命令即状态
    int32_t freqKHz;              //!< 激光器频率
    int32_t ampPercent;          //!< 激光器功率
    int32_t optingTemp;          //!< 激光器温度
    int32_t reserved[11];  //total 60 Byte
} SCDCMD_LASER_S;

//! 设备控制参数
typedef struct {
    SCDCMD_ENVIR_S envCtrl;      //! 环境命令/状态
    SCDCMD_CAMERA_S cmrCtrl;      //! 相机命令/状态
    SCDCMD_SCANER_S scnCtrl;      //! 扫描电机命令/状态
    SCDCMD_TURRET_S stgCtrl;      //! 转台命令/状态
    SCDCMD_LASER_S lsrCtrl;      //! 激光器命令/状态
    SCDCMD_PROGRM_S prgCtrl;      //! 扫描配置命令/状态 测距范围、回波类型
    SCDCMD_TIMSYN_S synStat;      //! 时间同步状态
    SCDCMD_DMI_S dmiData;        //! DMI 参数
    SCADA_DATAFUN_S storeData;     //数据存储
    USBCMR_TRIG_S usbCmrData;
    SCDCMD_CAMERA_S pmCmrData;     //! palm相机命令/状态
    SCDCMD_PALM_S pmDate;        //! palm 数据状态
    SCDCMD_PALMSTG_S pmStgData;     //! palm 转台数据
    int8_t reserve[244];                //! total 1KB
} SCADA_CONTROL_S;

typedef struct {
    SCDCMD_SCANER_S scnCtrl;      //! 扫描电机命令/状态
    SCDCMD_TURRET_S stgCtrl;      //! 转台命令/状态
    SCDCMD_LASER_S lsrCtrl;      //! 激光器命令/状态
    SCDCMD_PROGRM_S prgCtrl;      //! 扫描配置命令/状态
} SCADA_CONFIGCTR_S;

typedef struct {
    SCADA_CONFIGCTR_S ctlPara[SCDSCAN_TYPE_COUNT]; //720Byte
    int8_t reserve[304];
} SCADA_DEFAULTCTR_S;

#pragma pack(pop)

#pragma pack(push, 4)
//IMP文件格式_v3.1.0.txt
struct Imp_MetaData_S {
    SCADA_META_S DataHead;  //设备信息(1KB)
    SCADA_CONFIG_S scada_config;  //系统配置信息(2KB)
    SCDCMD_ENVIR_S scada_env;    //系统状态信息(2KB)
    int8_t revEnv[1988];
    SCADA_DEFAULTCTR_S scada_defCtrl; //设备默认控制参数信息(1KB)
    SCADA_CONTROL_S scada_control;  //设备控制参数(1KB)
    int8_t reservedA[1024]; //(1KB)
    SCDCAL_ARRAY_S scada_calib; //模型校验参数(2KB)
    ALGSC_ARRAY_S scada_algscA; //预处理模块配置信息(4KB)
    RANGE_CALIB_S scada_rangeCal; //距离灰度改正表(64KB)
    int8_t reservedB[2052]; //(2KB)
};

#pragma pack(pop)

class __star_export Metadata : public detail::basic_message<uint16_t, Imp_MetaData_S> {
public:
    uint64_t dataId() const override;
};

}
}
}
}


#endif //__STAR_SDK_IMP_HEADER_H
