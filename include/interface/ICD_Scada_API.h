/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
#ifndef ICD_SCADA_API_H_
#define ICD_SCADA_API_H_
#include "ICD_Common.h"
#include "ICD_LiDAR_API.h"
#include "ICD_UsbCamera.h"
#include <istoolkit/icnLiDARAPI.h>
#include "ICD_LiDAR_PRE.h"

#pragma pack(2)
typedef enum {
  eScadaPci ,
  eScadaNet ,
  eScadaUrt ,
  eScadaSim ,
  eScdSimDa ,
  eScdSimIm ,
  eScadaUsb ,
  eScadaUDP,  //udpRfans
  eScadaUdpPalm,
  eScadaGrdUdp, //v+AP 地面终端
  eScadaSN,//starnavi
} SCADA_TYPE_E ;

typedef enum {
  eDevCmdIdle = 0,
  eDevCmdWork ,
  eDevCmdSimu ,
  eDevCmdBreak ,
  eDevCmdReset ,
  eDevCmdAsk , //查询指令
} SCDEV_CMD_E ;

typedef enum {
  eNoMode  = 0,
  eFarMode ,
  eNearMode
} SCADA_RANGE_S ;

typedef enum{           //字符串已接收消息ID
  eRevMsgLaser   = 0x01,
  eRevMsgScanner = 0x02,
  eRevMsgEnv     = 0x04,
  eRevMsgProgam  = 0x08,
  eRevMsgPCM     = 0x10,
  eRevMsgPDT     = 0x20,
  eRevMsgPST     = 0x40,
  eRevMsgHeader  = 0x80,
  eRevFinish     = 0xFF
}SCDID_MESREV_E;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                           激光器
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
const int SCDLSR_POWR_DFLT = 100 ;          //!< 激光器功率默认值：100%
const int SCDLSR_POWR_MIN  = 50 ;
const int SCDLSR_POWR_MAX  = 100 ;
const int SCDLSR_FREQ_DFLT = 200 ;         //!< 激光器频率默认值：200KHz
const int SCDLSR_FREQ_MIN  = 30 ;
const int SCDLSR_FREQ_MAX  = 500 ;         //!< 500 kHz

//! laser
typedef struct {
  SCDEV_CMD_E cmdstat ;     //!< 激光器命令即状态
  int freqKHz;              //!< 激光器频率
  int ampPercent ;          //!< 激光器功率
  int optingTemp ;          //!< 激光器温度
  int reserved[11] ;  //total 60 Byte
} SCDCMD_LASER_S;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                            扫描电机
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

// 旋转模式
typedef enum {
  eScanPlusPosA,            //!< 绝对位置模式, 正向旋转
  eScanPlusPosR,            //!< 相对位置模式, 正向旋转
  eScanMinusPosA,           //!< 绝对位置模式, 反向旋转
  eScanMinusPosR,           //!< 相对位置模式, 反向旋转
  eScanPlusSpeed,           //!< 速度模式, 正向旋转
  eScanMinuSpeed            //!< 速度模式, 反向旋转
} SCDSCN_MODE_E;

// 扫描电机转稳状态
typedef enum {
  standReady = 0x0 ,  //准备状态
  eScanStand = 0x1 ,  //转稳状态
  eScanError = 0x2,
}SCDSCN_STATE_E ;

const int SCDSCN_SPEED_DFLT     = 1800;       //!< 电机速度默认值：1800
const int SCDSCN_SPEED_MIN      = 0;
const int SCDSCN_SPEED_MAX      = 6000;

//! scanner
typedef struct{
  SCDEV_CMD_E cmdstat ;
  SCDSCN_MODE_E mode ;
  float angSpeed ;           //!< depends on SCDSCN_MODE_E
  unsigned short roistr[4], roistp[4] ;
  unsigned int speedCount ;   //扫描电机count
  SCDSCN_STATE_E speedState ;  //扫描电机转速状态
  int grade;
  short reserved[10] ;  //total 60 Byte
} SCDCMD_SCANER_S;

//! motor hardValue
typedef struct{
  unsigned int spdTmpc ;
  unsigned int amcSpdCount ;
  unsigned int amcCrcValue ;
  int reserved[54] ; //total 60 Byte
} SCDMACH_SCANER_S;



// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                            转  台
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

// 控制默认值
const int SCDSTG_ANGLE_MIN    = 0;
const int SCDSTG_ANGLE_MAX    = 360;
const float SCDSTG_SPEED_MIN = 0.5 ;   // degree/sec
const float SCDSTG_SPEED_MAX = 3.0 ;   // degree/sec
const float SCDSTG_SPEED_DFLT = 3.0 ;  // degree/sec

//!< AMC电机状态
typedef enum {
  eAMCRdy = 0x0 ,
  eAMCRst,
  eAMCRsted,
  eAMCSpeedup,    //速度加速模式
  eAMCSpdWork,      //速度加速
  eAMCJogo,       //
  eAMCPst ,       //位置模式
  eAMCPsted ,     //位置模式完成
  eAMCCtnPsted ,  //连续位置模式完成
  eAMCWaitScaner,
  eAMCNxtPst,
  eAMCError,
  eAMCSetSpeed,
  eAMCCheckSpeed,
  eAMCUpdataFilter,
} SCDAMC_STATUS_E ;

//!< 转台子状态
typedef enum {
  eTurretNormal     = 0x0,
  eTurretReadyArea  = 0x1,
  eTurretSpeedArea  = 0x2,
  eTurretScanArea   = 0x3,
  eTurretStopArea   = 0x4,
  eTurretReadyPos   = 0x5,
  eTurretTrigePos   = 0x6,
  eTurretWorkePos   = 0x7,
  eTurretStopPos    = 0x8,
  eTurretCheckError = 0xE,
  eTurretSystemBusy = 0xF,
  eTurretWaitScaner,
  eTurretCmrTirgOk,
  eTurretPCChkError,
} SCDTURRET_MISTATE_E ;

//!< 转台主状态
typedef enum {
  eTurretInit     = 0x1,
  eTurretHold     = 0x2,
  eTurretDisable  = 0x3,
  eTurretRun      = 0x4,
  eTurretError    = 0x5,
} SCDTURRET_MASTATE_E ;

typedef struct {
  SCDTURRET_MISTATE_E currMiStat ;
  SCDTURRET_MASTATE_E currMaStat ;
}SCDBCD_MOTORSTATE_S;

const float TRIGGER_IMAGE_SPEED = 1; //DPS
//! turret
typedef struct{
  SCDEV_CMD_E cmdstat ;
  SCDTURRET_MODE_E mode ;
  float angSpeed ;
  float strAngle , stpAngle;          //!< 区域起始角度
  float currAng ;                     //!< 当前角度
  unsigned short roistr[4], roistp[4] ;
  SCDTURRET_MISTATE_E currMiStat ;
  SCDTURRET_MASTATE_E currMaStat ;
  int reserved[3] ;  //total 60 Byte
} SCDCMD_TURRET_S;


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                            步进电机
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
typedef struct{
  SCDEV_CMD_E cmdstat ;
  SCDSCN_MODE_E mode ;//旋转方向
  float angle;//转动角度
  float currAng;//当前角度 绝对位置编码器当前角度
  int reserved[11] ; //total 60 Byte
}SCDCMD_STEPMOTOR_S;


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                            环   境
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

//! 设备环境配置
typedef struct {
  GPS_TYPE_E gpsType ;      //!< GPS类型默认值：eGpsTrimble
  unsigned int utcTime ;    //!< 年月日时
  unsigned int utcSec ;     //!< UTC周秒
  bool pps ;               //!< PPS信号 0 :无信号   非零：有信号
  unsigned char satelliteCount ;    //!< gps 卫星个数
  unsigned char imuAmSign ;         //!< IMU 数据异常报警信号 0: 无报警信号  非零：有报警
  unsigned char imuDataState  ;     //!< IMU 数据传输状态 0: 数据停止传输  非零：数据传输
  float fGPSx ;             //!< 纬度
  float fGPSy ;             //!< 经度
  float fGPSz ;             //!< 海拔
  float temp[2];            //!< 温度
  float hemi[2];            //!< 湿度
  short altitude ;          //!< 高度
  unsigned short airspeed ; //!< 空速
  float incX ;              //!< inclinometer X
  float incY ;              //!< inclinometer Y
  float compassAng;         //!< 指南针角度
} SCDCMD_ENVIR_S  ;

// added by zhubing 2020.2.17
typedef struct  
{
	double mf_h;
	double mf_v;
	double mf_Dcol;
	float  _mf_freq;
	float _mf_cycle;
	char reserved[28]; // total 60 Byte
}SCDCMD_MF_S;


// 控制默认值
const int     SCDCMR_TIMER_MIN = 0;
const int     SCDCMR_TIMER_MAX = 6553;
const int     SCDCMR_TIME_DFLT = 4000;     //!< 时间曝光默认值：4000ms
const int     SCDCMR_ANGLE_MIN = 0;
const int     SCDCMR_ANGLE_MAX      = 360;
const int     SCDCMR_ANGLE_DFLT     = 180; //!< 角度曝光默认值：180度
const int     SCDCMR_ANGLOFFST_DFLT = 0;   //!< 角度偏移默认值：0

//默认值为零 使用lidar.cfg中的配置参数
const int     SCDCMR_DMI_ENCODER = 0 ;    //!< DMI编码器线数
const float     SCDCMR_DMI_CIRCUM = 0 ;  //!< DMI 周长

const int CMR_A_INDEX = 0;
const int CMR_B_INDEX = 1;

#define CMR_NUMBER 6 // 1 trige + 6 flash
typedef struct {
  SCDEV_CMD_E cmdStat ;        //!< 相机控制
  SCDCMR_MODE_E triggerMode ;  //!< 曝光模式
  int interval ;               //!< 时间/角度 间隔
  int angOffset ;              //!< 角度偏移
  unsigned int msgCount;       //!< 命令编号
  unsigned int flashCount[CMR_NUMBER] ;
  unsigned int trigCount ;

  float strAngle , stpAngle;   //!< 区域起始角度
  int reserved[1] ;            // total 60 Byte
} SCDCMD_CAMERA_S ;


//!< USB相机触发，从ICD_UsbCamera.h 中转移到这里定义，解决头文件自引用问题

const float USBCMR_EXPVALUE_DEF = 2000.0f;
const int USBCMR_A_INDEX = 0 ;
const int USBCMR_B_INDEX = 1 ;
typedef struct {
  SCDEV_CMD_E cmdStat ;         //!< USB相机控制
  SCDCMR_MODE_E triggerMode ;   //!< 曝光模式
  float  currAngle ;            //!< 当前转台trigger角度
  float itvAngle ;              //!< 曝光间隔角度
  float strAngle , stpAngle;    //!< 区域起始角度
  bool trigger[USBCmr_MAXNUMBER] ;
  float expValues[USBCmr_MAXNUMBER] ; //曝光时间
  unsigned short grayValue[USBCmr_MAXNUMBER] ; //灰度值
  short reserved[11] ;    //total 60 Byte
} USBCMR_TRIG_S ;

typedef struct {
  bool pps;           //!< PPS信号
  bool utc;           //!< UTC信号
  bool gps;           //!< GPS信号
  char reservedA[1] ; //total 60 Byte
  unsigned int tZero ;
  CSYSTEMTIME syncTime ;
  char reserved[36] ; //total 60 Byte
} SCDCMD_TIMSYN_S ;

//palm 数据传输参数
typedef struct {
  SCDEV_CMD_E cmdStat ;         //数据传输使能
  SCDEV_CMD_E imuState ;        //imu数据打包

  SCDEV_CMD_E gpsState ;        //gps数据打包

  unsigned int dmiMileage ;    //dmi 里程数 单位m

  GPS_TYPE_E gpsType ;       //GPS类型
  unsigned int utcTime ;    //UTC时间秒
  unsigned int utcYMDH;   //UTC年月日时
  unsigned short stateCount;  //卫星数
  unsigned short ppsSign  ;   //!< PPS信号  0 :无信号   1：有信号
  unsigned short imuUtcSync ;    //imu UTC时间同步 0 :未同步   1：同步
  unsigned short imuDataState ;  //imu 数据传输状态 0: 数据停止传输  非零：数据传输
  unsigned short gpsDataSate ;  //gps 数据传输状态 0: 数据停止传输  非零：数据传输
  unsigned short switchStat;    //按键状态
  unsigned int issDataSize;
  char reserved[16] ; //total 60 Byte
}SCDCMD_PALM_S ;

//palm 转台模式
typedef enum {
  eStageSpeed,  //速度模式
  eStageAngle,  //角度模式
}SCDMODE_PALMSTG_E;

//palm 转台方向
typedef enum {
  eStagePlus ,  //正
  eStageMinus,  //负
}SCDDIRE_PALMSTG_E;

//palm 转台
typedef struct {
  SCDEV_CMD_E cmdStat ;        //转台使能
  SCDMODE_PALMSTG_E mode ;     //转台工作模式
  float speed ;                //角度(度)/速度(dps)
  float angle ;                //转动角度
  SCDDIRE_PALMSTG_E dire ;     //方向
  float pulseAng ;             //每个脉冲对应的角度
  SCDEV_CMD_E cmrStat ;        //相机使能
  float crtAngle ;               //当前转动角度
  char reserved[28] ;          //!<total 60 Byte
}SCDCMD_PALMSTG_S ;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                           数据采集
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
#define MPMODE_REVISE_RANGE (120)  //!< MP模式过滤杂点数据 单位m
typedef enum {
  eProgrmModeNon = 0 ,
  eProgrmModeMP = 1 ,
}SCDMODE_PROGRAM_E;

#define ROI_NUMBER (2)

typedef enum {
  eNetFlow = 0x01,
  eDeviceSave = 0x02 ,
  ePCSave = 0x04 , //?
  eUSBSave = 0x08,
  eMemorySave = 0x10,
}SCDMODE_DATAPLACE_S;

typedef struct {             //同时设置数据采集模式0
  SCDEV_CMD_E cmdstat ;
  ECHO_TYPE_E m_eEcho ;      //!< 回波模式
  int rangemin, rangeMax ;   //!< meters
  SCDMODE_PROGRAM_E mpMode ;
  unsigned short isNetFlow ; //storageSetup()->setup 1控制设备（PC，PAD）存储；2: 设备存储；8: U盘存储
  short roiSstr[ROI_NUMBER], roiSstp[ROI_NUMBER] ;
  short roiTstr[ROI_NUMBER], roiTstp[ROI_NUMBER] ;
  unsigned short itvWorkTime ;   //设备间隔工作时间 单位秒
  char reserved[6] ;             //!<total 60 Byte
  LiDARImp_Action_E fileState ;
  DATA_TYPE_E dataType ;       //storageSetup()->setup
  SCDSCAN_TYPE_E scanMode ;    //近中远模式
  unsigned short resetSign ;    //重置RW 脉冲计数
} SCDCMD_PROGRM_S ;


//! 寄存器访问
typedef struct {
  unsigned int regAddr;          //!< 寄存器地址
  unsigned int regValue;         //!< 寄存器数据
  int reserved[13] ;             //!<total 60 Byte
} SCDCMD_REGIST_S;



typedef struct {
  LDRPROG_STAT_E state ;
  unsigned short vol, amp ;       //!< 工作电压，电流
  unsigned short humidA ;         //!< 湿度A
  unsigned short tempA;           //!< 温度A
  unsigned short humidB;          //!< 湿度B
  unsigned short tempB;           //!< 温度B
  unsigned short altitude ;       //!< 工作高度
  unsigned int tdcaFact ;        //!< 精度系数A  FPGA 寄存器返回  liyp
  unsigned int tdcbFact ;        //!< 精度系数B  FPGA 寄存器返回  liyp
  unsigned short mSet ;           //!< 单回波精度系数  FPGA 寄存器返回 liyp
  char reserved[32] ;             //!<total 60 Byte
} SCADA_STATE_S;

typedef struct {
  SCDEV_CMD_E cmdstat ;
  int reserve[14] ;  //total 60 Byte
} SCADA_POST_S ;

//!< FPGA 状态监控
typedef struct {
  unsigned char hardState ;    //!< T0脉冲
  unsigned char packState ;  //!< 数据打包
  unsigned char tranState ;  //!< 数据传输
  unsigned char simuState ;  //!< 仿真测试模式
  int reserve[14] ;  //total 60 Byte
} SCADA_FPGA_STATE ;


const SCDMODE_DATAPLACE_S SCDCMD_STRGPLACE_DFLT = eNetFlow ;
const DATA_TYPE_E SCDCMD_STRGTYPE_DFLT = eIMPDATA ;

typedef struct {
  SCDEV_CMD_E cmdstat ;
  SCDMODE_DATAPLACE_S storePlace ; //数据存储位置
  DATA_TYPE_E storeType ;          //数据存储类型
  int reserve[12] ;  //totao 60 Byte
} SCADA_DATAFUN_S ;

typedef struct {
  SCDEV_CMD_E cmdstat ;
  int dmiEncoder ;             //!< DMI 编码器线数
  float dmiCircum ;            //!< DMI 周长 m
  int dmiSelect ;
  int reserve[11] ;  //totao 60 Byte
} SCDCMD_DMI_S ;

typedef struct {
  DEVICE_TYPE_E  deviceType; //设备类型
  unsigned int   deviceRev;     //!< 设备编号
  int mpiaMode;
  int nRangScopeMin,  nRangScopeMax;            //!< scan distances （m 米）
  int	nLaserFreqMin,    nLaserFreqMax;            //!< e.g (50 - 300))Frequence (KHz)
  int nLaserPowerMin,   nLaserPowerMax;           //!< e.g Min(51%) (功率单位 %）
  int fScannerSpeedMin, fScannerSpeedMax;         //<! rpm
  float fStageSpeedMin, fStageSpeedMax;           //!< 转台速度边界
  int cmrCount;                                   //!< 外置相机个数
  int reserve[1] ;  //totao 60 Byte
} SCDCFG_LIMIT_S;
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// *                           设备状态
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
const bool SCDCMD_PPS_DFLT = false;         //!< PPS信号默认值：无信号
const bool SCDCMD_UTC_DFLT = false;         //!< UTC信号默认值：无信号
const bool SCDCMD_CMR_DFLT = false;         //!< Shutter信号默认值：无信号
const bool SCDCMD_GPS_DFLT = false;         //!< GPS信号默认值：无信号

//! 消息格式
//! liyp：结构体大小改变时必须修改 MSG_BUFFER_SIZE 大小
typedef struct cmdRpt{
  LiDAR_MSGID_E cmdId;               //!< 命令号
  int isAsk ;
  union {
    SCADA_STATE_S    statData ;      //! 设备状态
    SCDCMD_ENVIR_S   envData ;       //! 环境命令/状态
    SCDCMD_CAMERA_S  cmrData ;       //! 相机命令/状态
    SCDCMD_SCANER_S  scnData ;       //! 扫描电机命令/状态
    SCDCMD_TURRET_S  stgData ;       //! 转台命令/状态
    SCDCMD_LASER_S   lsrData ;       //! 激光器命令/状态
    SCDCMD_PROGRM_S  prgData ;       //! 扫描配置命令/状态
    SCDCMD_TIMSYN_S  synData ;       //! 时间同步状态
    SCDCMD_REGIST_S  regData ;       //! 寄存器读写
    USBCMR_CTRL_S    umrData ;       //! USB相机命令/状态
    USBCMR_TRIG_S    umrTrig ;       //! USB相机曝光
    SCDCMD_STEPMOTOR_S stepMtoData ; //! 步进电机命令/状态
    LiDAR_DATAINFO_S infoData;       //! 实时数据采集信息
    SCADA_FPGA_STATE fpgaData ;      //! FPGA状态
    SCADA_DATAFUN_S  storageData ;   //! 存储方式
    SCDCMD_DMI_S     dmiData ;        //! DMI 参数
    SCDCMD_CAMERA_S  pmCmrData ;     //! palm相机命令/状态
    SCDCMD_PALM_S    pmDate ;        //! palm 数据状态
    SCDCMD_PALMSTG_S pmStgData ;     //! palm 转台数据
    SCDCFG_LIMIT_S   limitData ;
// added by zhubing 2020.2.17
	SCDCMD_MF_S		 mfData;
    char msgstream[60] ;
  };
  cmdRpt() {
    isAsk = false ;
    cmdId = eMsgSoftCk ;
  }
} SCADA_CMDRPT_U ;

#define SCADA_CMD_COUNT (32)
typedef struct _CMDRPTS{
  SCADA_CMDRPT_U cmds[SCADA_CMD_COUNT] ;
  int count ;
  _CMDRPTS(){
    count = 0 ;
  }
}SCADA_CMDRPTS_S ;

//! 设备控制参数
typedef struct {
  SCDCMD_ENVIR_S      envCtrl ;      //! 环境命令/状态
  SCDCMD_CAMERA_S     cmrCtrl ;      //! 相机命令/状态
  SCDCMD_SCANER_S     scnCtrl ;      //! 扫描电机命令/状态
  SCDCMD_TURRET_S     stgCtrl ;      //! 转台命令/状态
  SCDCMD_LASER_S      lsrCtrl ;      //! 激光器命令/状态
  SCDCMD_PROGRM_S     prgCtrl ;      //! 扫描配置命令/状态 测距范围、回波类型
  SCDCMD_TIMSYN_S     synStat ;      //! 时间同步状态
  SCDCMD_DMI_S        dmiData ;        //! DMI 参数
  SCADA_DATAFUN_S     storeData ;     //数据存储
  USBCMR_TRIG_S       usbCmrData;
  SCDCMD_CAMERA_S     pmCmrData ;     //! palm相机命令/状态
  SCDCMD_PALM_S       pmDate ;        //! palm 数据状态
  SCDCMD_PALMSTG_S    pmStgData ;     //! palm 转台数据
  char reserve[244] ;                //! total 1KB
} SCADA_CONTROL_S ;



typedef struct {
  SCDCMD_SCANER_S     scnCtrl ;      //! 扫描电机命令/状态
  SCDCMD_TURRET_S     stgCtrl ;      //! 转台命令/状态
  SCDCMD_LASER_S      lsrCtrl ;      //! 激光器命令/状态
  SCDCMD_PROGRM_S     prgCtrl ;      //! 扫描配置命令/状态
} SCADA_CONFIGCTR_S ;

typedef struct {
  SCADA_CONFIGCTR_S ctlPara[SCDSCAN_TYPE_COUNT] ; //720Byte
  char reserve[304] ;
}SCADA_DEFAULTCTR_S ;

///  大彩串口屏协议相关定义

#define CMD_MODE_DISTANCE_NEAR 0x01
#define CMD_MODE_DISTANCE_MIDL 0x02
#define CMD_MODE_DISTANCE_FARR 0x03
#define CMD_SCAN_SPEED 0x06
#define CMD_TURRET_SPEED 0x08
#define CMD_TURRET_WORKMODE 0x09
#define CMD_TURRET_RESSET 0x03
#define CMD_LIDAR_START 0x00
#define CMD_LIDAR_STOP 0x01
#define CMD_CMR_START 0x02
#define CMD_CMR_STOP 0x03
#define CMD_TURRET_RESET 0X04
#define CMD_TURRET_RESET_STOP 0X05//006未添加
#define CMD_CMR_ANGLE_INTERVAL 0x06
#define CMD_CMR_ANGLE_BEGIN 0x07 //协议未说明
#define CMD_CMR_ANGLE_END 0x08
#define CMD_LASER_MODE 0x03

#define STATE_TYPE_MIN_DISTANCE		0x08 //测距最小距离
#define STATE_TYPE_MAX_DISTANCE		0x09 //测距最大距离
#define STATE_TYPE_MIN_REFLECT		0x0A //最小反射强度
#define STATE_TYPE_MAX_REFELEC		0x10 //最大反射强度
#define STATE_TYPE_TRANS_RATE		0x0B //传输速率
#define STATE_TYPE_DATA_TOTAL		0x0C //数据总量
#define STATE_TYPE_TEMPERATURE		0x0F //温度

enum UartScreenStatus{
	US_IDLE,
	US_INIT,
	US_LASER_STATUS_WAITING,
	US_SCANER_STATUS_WAITING,
	US_TURRET_STATUS_WAITING,
	US_LASER_MODE_WAITING,
	US_SCAN_SPEED_WAITING,
	US_TURRET_SPEED_WAITING,
	US_TURRET_MODE_WAITING,
	US_CMR_AGL_BEGIN_WAITING,
	US_CMR_AGL_INTV_WAITING,
	US_CMR_AGL_END_WAITING,
	US_WORKING
};

enum UartDeviceType {
	UART_TYPE_NONE,
	UART_TYPE_LIDAR,
	UART_TYPE_CMR,
	UART_TYPE_TURRET_RESET
};

enum UartTurretMode {
	UART_TURRET_MODE_CNTU,
	UART_TURRET_MODE_180D,
	UART_TURRET_MODE_360D
};

enum LaserStatus {
	LASER_STATUS_IDLE,
	LASER_STATUS_WORKING,
	LASER_STATUS_SIMU
};

enum ScanerStatus {
	SCANER_STATUS_IDLE,
	SCANER_STATUS_WORKING
};

enum TurretStatus {
	TURRET_STATUS_IDLE,
	TURRET_STATUS_WORKING
};

struct UartSceenParam {
	UartDeviceType deviceType;
	//device mode
	LaserStatus laserStatus;
	ScanerStatus scanerStatus;
	TurretStatus turretStatus;
	// lidar 
	SCDSCAN_TYPE_E distanceMode; // 近中远
	char scanSpeed[8];
	char turretSpeed[8];
	bool turretReseting;
	UartTurretMode turretMode;//continue, 180, 360
	//cmr
	int cmrAglBegin;
	int cmrAglEnd;
	int cmrAglIntv;
};


#pragma pack()
#endif
