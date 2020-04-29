/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
#ifndef ICD_LIDAR_API_H_
#define ICD_LIDAR_API_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ICD_LiPRE_API.h"
#include "ICD_LiDAR_PRE.h"
#pragma pack(2)
#include <time.h>
const int IMP_HEADER_SIZE = 0x14000;    //IMP文件头大小 80KB

const int END_LEFT_SIZE = 0xA00000;    //IMP文件头大小 131KB

const char LIDAR_IPADDR_DFLT[] = "192.168.0.188" ;  //!< 默认IP：192.168.0.188
const char LIDAR_IPADDR_WIFI[] = "192.168.10.1" ;   //!< 无线控制 IP：192.168.10.1
const char RFANS_IPADDR_DFLT[] = "224.0.0.1" ;  //!< 默认IP：192.168.0.188
const int LIDAR_MSGPORT = 2008;  //消息数据端口
const int LIDAR_PNTPORT = 2009;  //快数据端口
const int LIDAR_IMGPORT = 2011;  //图像数据端口
const int LIDAR_SLWPORT = 2012;  //慢数据端口
const int LIDAR_UDPPORT = 2013;  //UDP CMD 端口

//added by zhubing for getGPS INFO
const int LIDAR_GPSPORT = 2020;  //导航数据传输端口

const int RFANS_DATA_UDPPORT = 2014 ;
const int RFANS_CMD_UDPPORT = 2015;

const int LIDAR_GPSMSG_UDPPORT = 2016;  //端口


const int SCADA_UDP_CPORT = 2017 ;// 本地UDP端口号

const int NET_SLOW_TIMEOUT = 1000 ; //微秒
const int CHNL_OPEN_ID = 7 ;        //socket
const int IMPFILE_CLOSE_COUNT = 50 ;
const int Y_M_D_SIZE = 9; //size of year,month ,day string; including
const int H_M_S_SIZE = 7; //hour, minute,second string; including 0

static const int DATA_FILE_SIZE = 0x8000000; // 128 * 1024 * 1024 =128M Bytes
static const int USBDATA_FILE_SIZE = 0x40000000; // 128 * 1024 * 1024 =1GM Bytes
static const int SLOWDATA_SIZE = 0x100000;  // 1M Bytes
static const int SLOW_FILE_SIZE = 0x20000000;  // 512M Bytes

typedef enum {
  eAPilot = 0 ,
  eRAngle,
  eUArm,
  eTPilot,
  eAK,
  eRFans,
  eSL,
  eCFans,
  eUT = 8 ,
	IRA,
} DEVICE_TYPE_E;

typedef enum {     //!<回波类型
  eEchoAll = 0,
  eEchoOne = 1,    //!< 1回波
  eEchoTwo = 2,    //!< 2回波
  eEchoTri = 4,    //!< 3回波
  eEchoFor = 8,    //!< 4回波
} ECHO_TYPE_E ;

const int SCDSCAN_TYPE_COUNT = 3 ;

typedef enum {     //!<扫描采集类型
  eScanNear = 0,   //!< 近距离模式
  eScanMiddle = 1, //!< 中离模式
  eScanMiddleFar = 2, //!< 中远距离模式
  eScanFar = 3,    //!< 远距离模式
} SCDSCAN_TYPE_E ;

typedef enum {               //!< 相机曝光模式
  eCmrTimerNero,             //!< 时间曝光, 无零位触发
  eCmrTimerZero,             //!< 时间曝光, 有零位触发
  eCmrAngleNero,             //!< 角度曝光, 无零位触发
  eCmrAngleZero,             //!< 角度曝光, 有零位触发
  eCmrPosnNero,              //!< 位置曝光, 无零位触发
  eCmrPosnZero,              //!< 位置曝光, 有零位触发
  eCmrRangeNero,             //!< 距离曝光, 无零位触发
  eCmrRangeZero,             //!< 距离曝光, 有零位触发
} SCDCMR_MODE_E;


typedef enum {             //!< 转台工作模式
  eTurretReset     = 0x00, //!< 重置
  eTurretContinue  = 0x01, //!< 连续模式
  eTurretArea      = 0x02, //!< 区域模式
  eTurretPosition  = 0x04, //!< 位置模式
  eTurretNextTrige = 0x08, //!< 下一个曝光点
  eTurretStop      = 0x10, //!< 停止
  eTurretSearch    = 0x20, //!< 查询
} SCDTURRET_MODE_E ;

typedef enum {        //GPS类型
  eGpsApplanix = 0,
  eGpsTrimble=1 ,
  eGpsNoval=2 ,
  eGpsJavat=3 ,
  eGpsNmea = 4,
  eGpsGsof = 5,
  eGPSInpps = 6,
} GPS_TYPE_E ;



typedef enum {//上位机的控制模式
  eLidarNet ,  // Acting as Server, accept cmd from Ethernet
  eLidarGui ,  // controlling the LiDAR through GUI
  eLidarUrt ,  // Acting as Server, accept cmd from Uart
  eDataSave ,  // for Data Handle: Save locally
  eDataFlow ,  // for Data Handle: Transfer through Net
  eLidarNetAndUrt,
  eLidarFlyUrt,
} LiDAR_TYPE_E ;

typedef enum {
  eFileStay, // no action to file handler
  eFileNew,
  eFileRenew,
  eFileClose,
  eFileCloseAll,
  eFileImpClose,
  eFileIssClose,
} LiDARImp_Action_E ;



/********** Structures ***********/
const int LiDAR_SHOTBUF_SIZE = 0x40000 ;  //256KB
const int LiDAR_RINGBUF_NUM = 1024;       //增大缓存为256MB

const int LiDAR_SHOTBUF_MINI_SIZE = 0x8000 ; //32KB 每条扫描线 3000 个点，以一回波数据大小计算 54KB
const int LiDAR_RINGBUF_MINI_NUM = 8192;     //32KB*8192 = 256MB

const int LiDAR_CALC_RINGBUF_NUM = 256;
typedef struct { //for PCI DMA, 256 MBytes in total
  int wrHead, rdTail ,bfSize;
  char * buffer[LiDAR_RINGBUF_NUM]; //[LiDAR_RINGBUF_SIZE] ;
  int ROW,COLUMN ; //liyp 内存的组织方式PCI interface: 1024*256KB; USB interface:8192*32KB
  int readCount ;    //一次读buffer的个数
} SHOTS_RINGBUF_S ;

inline int SHOTS_RINGBUF_COUNT(SHOTS_RINGBUF_S *mtRingBuff) {
  int tmpCount = 0;
  if (mtRingBuff->wrHead > mtRingBuff->rdTail) {
    tmpCount = mtRingBuff->wrHead - mtRingBuff->rdTail;
  } else if (mtRingBuff->wrHead < mtRingBuff->rdTail){
    tmpCount = mtRingBuff->ROW - mtRingBuff->rdTail + mtRingBuff->wrHead;
  }
  return tmpCount ;
}

const int SLOW_RINGBUF_SIZE = 0x80000;//1MB
const int SLOW_READ_SIZE = 0x4000 ; //16KB
const int SLOW_WRITE_SIZE = 0x40000 ; //256KB
const int SLW_DATA_RDCOUNT = 16 ;

const int SLOW_READ_SIZE_1K = 0x400; //1KB

typedef struct {
  int wrHead, rdTail;
  char buffer[SLOW_RINGBUF_SIZE];
} SLOW_RINGBUF_S ;
const unsigned int IMPHEAD_SYN_WORD = 0xA7A7A7A7;
const unsigned int IMG_SYN_WORD = 0xF9F9F9F9 ;
const unsigned int JPG_SYN_WORD = 0xFAFAFAFA ;
const unsigned int CMPIMG_SYN_WORD = 0xF8F8F8F8 ;
typedef struct {
  unsigned int synWord ;
  unsigned short cmrNo ;
  unsigned int imgSize ;
  float currAngle ;
} USBCMR_IMGHEADER_S ;

// buffer size

const int USBCmr_MAXNUMBER = 2 ;

const int IMG_WIDTH = 2592;
const int IMG_HEIGHT = 1944;
const int IMG_YUYBITS = 16;
const int IMG_SIZEFIX = 40;
const int IMG_HEAD_SIZE = 54 ;

const int IMG_SIZEMAX = sizeof(USBCMR_IMGHEADER_S) + (IMG_WIDTH*IMG_HEIGHT*3) +IMG_HEAD_SIZE ;


//const int CAMERA_IMGRBUF_SIZE = IMG_SIZEMAX ;
const int CAMERA_IMGRBUF_SIZE = 15*1024*1024 ;

const int CAMERA_RINGBUF_NUM = 2 ;
const int JPG_BUFFER_SIZE = (512*1024)  ;

typedef struct {
  int wrHead, rdTail , bfSize;
  int BUFFER_MAX_SIZE ;
  char * buffer[CAMERA_RINGBUF_NUM]; //
} IMAGE_RINGBUF_S ;

const int MSG_BUFFER_SIZE = 68;
typedef struct {
  int wrHead, rdTail ;
  char msgArray[LiDAR_RINGBUF_NUM][MSG_BUFFER_SIZE] ; //64 > sizeof(SCADA_CMDRPT_U)
} MESSG_RINGBUF_S ;

const int GPMSG_BUFFER_SIZE = 1024;

typedef struct {
  int wrHead, rdTail ;
  struct {
    char gpsMsg[GPMSG_BUFFER_SIZE] ;
    unsigned short msgSize ;
  } msgArray [LiDAR_RINGBUF_NUM];
} GPSMSG_RINGBUF_S ;

inline int LDRPROG_GPSMSG_RING_WRITE(GPSMSG_RINGBUF_S* msgRing ,char *mtMsg,int size) {
  char *tmpWrtMsg = NULL ;
  if( msgRing ) {
    tmpWrtMsg = msgRing->msgArray[msgRing->wrHead].gpsMsg ;
    memset(tmpWrtMsg, 0, GPMSG_BUFFER_SIZE) ;
    msgRing->msgArray[msgRing->wrHead].msgSize = 0 ;
    memcpy(tmpWrtMsg, mtMsg, size) ;
    msgRing->msgArray[msgRing->wrHead].msgSize = size ;
    msgRing->wrHead = (msgRing->wrHead+1) % LiDAR_RINGBUF_NUM ;
  }
  return size ;
}

inline int LDRPROG_GPSMSG_RING_READ(GPSMSG_RINGBUF_S *msgRing , char *mtReadMsg, int size)  {
  char *tmpMsg = NULL ;
  if(msgRing) {
    if(msgRing->wrHead !=  msgRing->rdTail) {
    tmpMsg = msgRing->msgArray[msgRing->rdTail].gpsMsg ;
    size = msgRing->msgArray[msgRing->rdTail].msgSize ;
    memcpy(mtReadMsg, tmpMsg, size) ;
    msgRing->rdTail = (msgRing->rdTail+1)% LiDAR_RINGBUF_NUM ;
    } else {
    size = 0 ;
    }
  }
  return size ;
}

inline int LDRPROG_GPSMSG_RING_INIT(GPSMSG_RINGBUF_S *msgRing) {
  if( msgRing ) {
  memset(msgRing, 0, sizeof(GPSMSG_RINGBUF_S) ) ;
  }
  return 0 ;
}

typedef enum {
  eLidarProgError ,
  eLidarProgStandBy ,
  eLidarProgReady ,
  elidarProgPre ,
  eLidarProgEnvir , //
  eLidarProgCamera=5 , //
  eLidarProgMotor , //
  eLidarProgStage , //
  eLidarProgLaser , //
  eLidarProgStart , //
  eLidarProgAction=10 ,  // eLidarProgReady <-> eLidarProgAction
  eLidarCmrScaning ,  // eLidarProgReady <-> eLidarCmrScaning
  eLidarCmrWaitImg1 ,
  eLidarCmrWaitImg2 ,
  eLidarCmrTrigOK,
  eLidarStopAll,
  eLidarProgBreak,
  eLidarCmrInit ,
  eLidarCmrInited ,
  eLidarCmrFinished,
  eLidarUCmrInit,
  eLidarUCmrScaning,
  eLidarUCmrWaiting,
  eLidarUCmrFinished,
  eLidarCheckStagStop,
  eLidarUCmrTrigger,
  eLidarCmrTrigger,
} LDRPROG_STAT_E ;

typedef enum
{
  eUsbDiskBreak,
  eUsbDiskInsert,
}LDRUSB_STATE_E;

typedef enum {
  eDFrameReady,
  eDFrameFinished,
  eDFrameReadChange,
}SL_DFRAME_E;

//设备状态报告
typedef struct {
  LDRPROG_STAT_E lsrState ;
  float  nRangeScopeMin_;
  float  nRangeScopeMax_ ;       //时间窗口
  int    nReflectMin_;
  int    nReflectMax_ ;          //激光的反射强度(0, 2047)；
  float  fDataCollectSpeed_ ;    //速率 KB/s;
  double fDataTotal_ ;           //数据总量 KB
  unsigned int   curImpSize_ ;           //当前IMP大小 KB
  int scadaFlag_ ;               //速率统计的位置： 1:下位机统计，0：上位机统计
  PRE_DATASTATE_E dataState;     //数据状态
  unsigned int diskSize ;        //磁盘空间剩余大小
  LDRUSB_STATE_E usbDiskStat;    //
  SL_DFRAME_E frameState;
  int reserve[2] ;               //total 60 Byte
} LiDAR_DATAINFO_S ;

/*
    字符串协议
*/
#define PARA_STR_LENGTH (64) //参数长度
#define CMD_STR_LENGTH (1024) //字符命令长度

#define DEV_MODE_HEAD "m"       //设备(激光/转台/相机/数据作业)模式
#define DEV_FREQ_HEAD "freq"     //设备(转台/扫描电机)速度
#define DEV_ROIS_HEAD "rois"     //设备扫描电机方向ROI
#define DEV_ROIT_HEAD "roit"     //设备转台电机方向ROI
#define DEV_REPORT_HEAD "a"       //反馈

//激光器
#define LSR_RUN_HEAD "LSR_RUN"
#define LSR_STP_HEAD "LSR_STP"
#define LSR_ASK_HEAD "LSR_ASK"

#define LSR_FREQ_HEAD   DEV_FREQ_HEAD  //!< 激光平率
#define LSR_POWER_HEAD  "power" //!< 激光功率
#define LSR_SIMU_HEAD  "simu"  //!< 仿真模式

//扫描电机
#define SCN_RUN_HEAD "SCN_RUN"
#define SCN_STP_HEAD "SCN_STP"
#define SCN_ASK_HEAD "SCN_ASK"

#define SCN_COUNT_HEAD "scncnt"   //!< 电机转过的圈数
#define SCN_STEADY_HEAD "scnstdy" //!< 电机速度转稳状态

#define SCN_SPD_HEAD "rpm"        //!< 扫描电机速度

//转台电机
#define STG_RUN_HEAD "STG_RUN"
#define STG_STP_HEAD "STG_STP"
#define STG_ASK_HEAD "STG_ASK"

#define STG_CURANG_HEAD "angle"
#define STG_SPD_HEAD    "dps"          //!< 转台速度
#define STG_MODE_HEAD DEV_MODE_HEAD  //!< 转台模式

//数据采集作业
#define TSK_RUN_HEAD "TSK_RUN"
#define TSK_STP_HEAD "TSK_STP"
#define TSK_BRK_HEAD "TSK_BRK"
#define TSK_ASK_HEAD "TSK_ASK"

#define TSK_MIN_HEAD      "min"           //!< 最小测距
#define TSK_MAX_HEAD      "max"           //!< 最大测距
#define TSK_RANGE_HEAD    "range"           //!< 测距
#define TSK_ECHO_HEAD     "echo"          //!< 回波类型
#define TSK_MP_HEAD       DEV_MODE_HEAD   //!< mp模式
#define TSK_NETFLOW_HEAD  "netflow"       //!< 网络流
#define TSK_ROIS_HEAD     DEV_ROIS_HEAD   //!< 扫描角度ROI
#define TSK_ROIT_HEAD     DEV_ROIT_HEAD   //!< 转台角度ROI
#define TSK_SYNCTIME_HEAD "syct"          //!< 同步时间信息格式
#define TSK_RESETSIGN_HEAD "rsts"         //!< 脉冲信号重置
#define TSK_DTYPE_HEAD     "dtype"       //!< 数据类型
#define TSK_ITVWTIME_HEAD   "itvtime"      //!< 间隔工作时间



//外置相机
#define CMR_RUN_HEAD "CMR_RUN"
#define CMR_STP_HEAD "CMR_STP"
#define CMR_ASK_HEAD "CMR_ASK"

#define CMR_MODE_HEAD DEV_MODE_HEAD  //相机模式
#define CMR_ROI_HEAD DEV_ROIT_HEAD   //角度ROI
#define CMR_ITV_HEAD DEV_FREQ_HEAD   //相机参数

//内置USB相机
#define UCM_RUN_HEAD "UCM_RUN"
#define UCM_STP_HEAD "UCM_STP"
#define UCM_ASK_HEAD "UCM_ASK"

#define UCM_MODE_HEAD DEV_MODE_HEAD //相机模式
#define UCM_ROI_HEAD DEV_ROIT_HEAD  //角度ROI
#define UCM_A_HEAD "cmra"           //相机A 拍照
#define UCM_B_HEAD "cmrb"           //相机B 拍照
#define UCM_AEXP_HEAD "expa"           //相机A 曝光值
#define UCM_BEXP_HEAD "expb"           //相机B 曝光值

// added by zhubing 2020.2.17
// MF 参数设置
#define MF_RUN_HEAD "MF_RUN"
#define MF_H_HEAD     "h"             //!< 航高
#define MF_V_HEAD     "v"             //!< 飞行速度
#define MF_DCOL_HEAD     "Dcol"       //!< 横向密度
#define MF_DROW_HEAD     "Draw"       //!< 纵向密度
#define MF_PULSES_HEAD   "Pulse"       //!< 光脉冲数
#define MF_FREQ_HEAD   "Frequency"       //!< 点频

//数据存储配置
#define STORAGE_RUN_HEAD "SRG_RUN"  //!< 存储配置
#define STORAGE_ASK_HEAD "SRG_ASK"  //!< 存储查询

//AK设备串口控制，参数通过lidar.cfg 文件获取
#define LDR_STR_HEAD "LDR_START"  //!< 启动扫描
#define LDR_STP_HEAD "LDR_STOP"   //!< 停止扫描
#define LDR_BRK_HEAD "LDR_BREAK"  //!< 暂停扫描

//DMI
#define DMI_RUN_HEAD "DMI_RUN"   //!< DMI参数配置
#define DMI_ASK_HEAD "DMI_ASK"   //!< DMI参数查询

#define DMI_CODE_HEAD "code"     //!< dmi 编码器线数
#define DMI_CIRC_HEAD "circ"     //!< DMI 车轮周长

//PALM相机
#define PCM_RUN_HEAD "PCM_RUN" //启动palm 相机
#define PCM_STP_HEAD "PCM_STP" //停止palm 相机
#define PCM_ASK_HEAD "PCM_ASK" //查询palm 相机

//PALM 数据传输
#define PDT_RUN_HEAD "PDT_RUN" //启动palm 数据采集
#define PDT_STP_HEAD "PDT_STP" //停止 palm 数据采集
#define PDT_ASK_HEAD "PDT_ASK" //查询 palm 状态


//PALM RT转台
#define PST_RUN_HEAD "PST_RUN" //启动palm 转台
#define PST_STP_HEAD "PST_STP" //停止palm 转台
#define PST_ASK_HEAD "PST_ASK" //停止palm 转台

#define PST_RES_HEAD "res"    //每个脉冲对应的角度
#define PST_ANTI_HEAD "anti"   //默认为正方向，带"-anti"为反方向
#define PST_ANGLE_HEAD "pangle"
#define PST_CURANG_HEAD "curang"  //palm 转台当前角度

#define LDR_ASK_HEAD "LDR_ASK" //设备状态查询
#define ENV_ASK_HEAD "ENV_ASK" //环境状态查询
#define CFG_ASK_HEAD "CFG_ASK" //设备配置信息查询
//
/**
            状态
**/
#define LDR_RPT_HEAD "LDR_RPT"    //!< 设备状态报告
#define LSR_RPT_HEAD "LSR_RPT"    //!< 激光器
#define SCN_RPT_HEAD "SCN_RPT"    //!< 扫描电机
#define STG_RPT_HEAD "STG_RPT"    //!< 转台
#define CMR_RPT_HEAD "CMR_RPT"    //!< 外置相机相机
#define UCM_RPT_HEAD "UCM_RPT"    //!< 内置USB相机
#define TSK_RPT_HEAD "TSK_RPT"    //!< 数据采集作业
#define ENV_RPT_HEAD "ENV_RPT"    //!< 环境 状态
#define DMI_RPT_HEAD "DMI_RPT"    //!< 环境 状态

#define PCM_RPT_HEAD "PCM_RPT"    //!< PALM 相机反馈
#define PDT_RPT_HEAD "PDT_RPT"    //!< PALM数据传输
#define CFG_RPT_HEAD "CFG_RPT"    //!< 配置信息反馈


/**
            状态参数
**/
#define CMR_TCOUNT_HEAD "tc"	//palm相机触发个数
#define CMR_FLASH1_HEAD "f1"	//palm相机1反馈个数
#define CMR_FLASH2_HEAD "f2"	//palm相机2反馈个数
#define CMR_FLASH3_HEAD "f3"	//palm相机3反馈个数
#define CMR_FLASH4_HEAD "f4"	//palm相机4反馈个数
#define CMR_FLASH5_HEAD "f5"	//palm相机5反馈个数
#define CMR_FLASH6_HEAD "f6"	//palm相机6反馈个数
#define DMI_COUNT_HEAD "dmiCount"	//dmi 里程数
#define IMU_SYNC_HEAD "imuSync"	//imu 时间同步信号
#define IMU_DATA_HEAD "imuData"	//imu 数据传输
#define IMU_WARN_HEAD "imuwarn"	//imu 警告
#define GPS_DATA_HEAD "gpsData"	//gps 数据传输
#define GPS_TYPE_HEAD "gpsType"	//gps类型
#define UTC_YMD_HEAD  "utc"	//utc时间
#define UTC_SEC_HEAD  "sutc"	//utc时间
#define SATE_COUNT_HEAD "sate"	//卫星数
#define PPS_SYNC_HEAD "pps"	//pps信号
#define GPS_X_HEAD "gpsx"	//纬度
#define GPS_Y_HEAD "gpsy"	//经度
#define GPS_Z_HEAD "gpsz"	//海拔
#define INCY_X_HEAD "incx"	//倾角仪x轴
#define INCY_Y_HEAD "incy"	//倾角仪y轴
#define TEMP_A_HEAD "tempA"	//温度传感器A
#define TEMP_B_HEAD "tempB"	//温度传感器B
#define HEMI_A_HEAD "hemiA"	//湿度传感器A
#define HEMI_B_HEAD "hemiB"	//湿度传感器B
#define AIR_HEIGHT_HEAD "altitude"	//气压高度
#define AIR_SPEED_HEAD "airspeed"	//空速
#define RANGE_MIN_HEAD "rangMin"	//距离最小值
#define RANGE_MAX_HEAD "rangMax"	//距离最大值
#define REF_MIN_HEAD "refMin"	//强度最小值
#define REF_MAX_HEAD "refMax"	//强度最大值
#define DATA_SPEED_HEAD "dataSpeed"	//数据采集速率
#define DATA_TOTAL_HEAD "dataTotal"	//数据总量
#define IMP_SIZE_HEAD "impSize"	//当前IMP大小
#define CMR_COUNT_HEAD "cmrc"	//命令编号
#define CMR_TRGA_HEAD "trigerA"	//相机A 触发时刻
#define CMR_TRGB_HEAD "trigerB"	//相机B 触发时刻
#define CMR_FLASHA_HEAD "flashA"	//相机A shutter信号时刻
#define CMR_FLASHB_HEAD "flashB"	//相机B shutter信号时刻
#define STATE_POWUP_HEAD "powup"	//上电
#define STATE_READY_HEAD "ready"	//准备
#define STATE_ERROR_HEAD "error" // 错误状态
#define STATE_WORKING_HEAD "working"	//工作
#define STATE_STOPED_HEAD "stoped"	//停止
#define STATE_PAUSE_HEAD "pause"	//暂停
#define STATE_SICK_HEAD "sick"	//生病状态
#define STATE_INIT_HEAD "init"	//转台初始化
#define STATE_FIXING_HEAD "fixing"	//转台定位中
#define STATE_FIXED_HEAD "fixed"	//转台定位完成
#define STATE_POSEND_HEAD "stpposm"	//转台位置模式结束
#define STR_IMU_HEAD "strimu"	//imu 数据采集
#define STR_GPS_HEAD "strgps"	//gps 数据采集
#define STP_IMU_HEAD "stpimu"	//停止imu数据采集
#define STP_GPS_HEAD "stpgps"	//停止gps数据采集

#define LIDAR_STATE_HEAD "state"  //状态
#define LIDAR_FLAG_HEAD "flag"  //状态
#define LIDAR_DATA_HEAD "data"  //状态
#define LIDAR_FRAME_HEAD "frame"  //状态

#define DEVICE_TYPE_HEAD "deviceType"
#define DEVICE_REV_HEAD "deviceRev"
#define FREQ_MIN_HEAD "freqMin"
#define FREQ_MAX_HEAD "freqMax"
#define POWER_MIN_HEAD "powerMin"
#define POWER_MAX_HEAD "powerMax"
#define SCNSPEED_MIN_HEAD "scnSpdMin"
#define SCNSPEED_MAX_HEAD "scnSpdMax"
#define STGSPEED_MIN_HEAD "stgSpdMin"
#define STGSPEED_MAX_HEAD "stgSpdMax"
//#define CMR_COUNT_HEAD "cmrCount"
#define MPIA_MODE_HEAD "mpiaMode"

#define DATA_BUTTON_HEAD "enDataBtn"  //palm数据采集按键使能

#define CMD_START_CHAR  '<'
#define CMD_STOP_CHAR   '>'
#define CMD_START_STRING  "<"
#define CMD_STOP_STRING  ">"
#define PARA_START_STRING "-"
#define PARA_VALUE_STRING  "="

#pragma pack()
#endif
