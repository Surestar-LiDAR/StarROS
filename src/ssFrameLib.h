/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _FRAME_LIB_H_
#define _FRAME_LIB_H_
#include <iostream>
#include <string>
#include "rfans_driver/RfansPacket.h"
#include <rfans_driver/point_types.h>
using namespace std;

const static int REG_DEVICE_CTRL = (0x2040);
const static int REG_DEVICE_CTRL_OLD = (0x0040);
const static int REG_DATA_TRANS_OLD = (0x70);
//C-FRANS128 date use
const static int REG_DATA_TRANS = (0x20D0);
const static int REG_DATA_LEVEL = 0x2008;
const static int REG_DATA_LEVEL_OLD = 0x0008;

const static int CMD_SCAN_SPEED_5HZ = 0;
const static int CMD_SCAN_SPEED_10HZ = 0x50;
const static int CMD_SCAN_SPEED_20HZ = 0xF0;
const static int CMD_SCAN_ENABLE = 0x1;
const static int CMD_LASER_ENABLE = 0x2;
const static int CMD_CALC_DATA = 0x2;
const static int CMD_DEBUG_DATA = 0x5;
const static int CMD_RCV_CLOSE = 0x0;
const static int CMD_LEVEL0_ECHO = 0x00000000 ;
const static int CMD_LEVLE0_DUAL_ECHO = 0x00000002;
const static int CMD_LEVEL1_ECHO = 0x01000200;
const static int CMD_LEVEL1_DUAL_ECHO = 0x01000202;
const static int CMD_LEVEL2_ECHO = 0x0201BF00;
const static int CMD_LEVEL2_DUAL_ECHO = 0x0201BF02;
const static int CMD_LEVEL3_ECHO = 0x03000100;
const static int CMD_LEVEL3_DUAL_ECHO = 0x03000102;



const static int ANGLE_SPEED_5HZ = 5;
const static int ANGLE_SPEED_10HZ = 10;
const static int ANGLE_SPEED_20HZ = 20;

static unsigned short DEVICE_PORT_NUMBER = 2014;
static unsigned short PC_PORT_NUMBER = 2014;
static std::string DEVICE_IP_STRING = "192.168.0.3";

static unsigned short UDP_FRAME_MIN =(18);

#define DEB_FRAME_WRITE (0xA5)  //write head sync
#define DEB_FRAME_READ  (0x5a)  //read head sync
#define DEB_FRAME_ERROR  (0xE7) //err  haad sync

#define FRAME_MSG_LENGTH (1024) 

const int UDPREG_MAX_COUNT = 256;
const int ROMREG_MAX_COUNT = 0x7FF;




typedef enum{
  eCmdWrite,  //write command
  eCmdRead,   //read command
  eCmdQuery,  //Query command
} SCD_FRAME_TYPE_E ;

typedef struct _frams_buffer{
  char msgStream[FRAME_MSG_LENGTH];
  int writeIdx;
  int readIdx;
  int length;
} FRAMS_BUFFER_S;

#pragma pack(1)
typedef struct {                 //! rfans udp command package
  unsigned char msgHead;         //!< head sync
  unsigned char msgCheckSum;     //!< check sum
  unsigned short regAddress;     //!< register add
  unsigned int regData;          //!< register data
} DEB_FRAME_S;

typedef enum {
  eDevCmdIdle = 0,
  eDevCmdWork,
  eDevCmdSimu,
  eDevCmdBreak,
  eDevCmdReset,
  eDevCmdAsk,
} DEB_CMD_E;

typedef enum {
  eFormatCalcData = 0x2,
  eFormatDebugData = 0x5,
}DEB_DFORMAT_E;


static int DEVICE_MOTOR_HZ = 5;
typedef struct {
  DEB_CMD_E cmdstat;
  DEB_DFORMAT_E dataFormat;
  int scnSpeed;
  int dataLevel;
  int lsrFreq;
  float rangeMin, rangeMax;
}DEB_PROGRM_S;

static const size_t upd_packet_size = 0x8000;  //32KB
const unsigned char ID_RFANSBLOCKV2_SYNC = 0x96;
const unsigned char ID_RFANSBLOCKV32_0_15_SYNC = 0x97;
const unsigned char ID_RFANSBLOCKV32_16_31_SYNC = 0x98;

const unsigned char ID_RFANSBLOCKV6G_0_15_SYNC = 0x99;
const unsigned char ID_RFANSBLOCKV6G_16_31_SYNC = 0x9A;

const unsigned char ID_RFANSBLOCKV32_GM_0_15_SYNC = 0x87;
const unsigned char ID_RFANSBLOCKV32_GM_16_31_SYNC = 0x88;

const unsigned char ID_RFANSBLOCK_GM_SYNC = 0xee;


const int UDP_PACKET_SIZE_V5A = 1380;
const int UDP_PACKET_SIZE_V6G = 1206;
const int UDP_PACKET_SIZE_DATA_LEVEL_ORI=1406;

typedef struct {
  unsigned short angle  : 16 ;          //scan angle, 0.01°
  unsigned short rangeOne  : 16 ;       //echo 1,  cm
  unsigned short rangeTwo  : 16 ;       //echo 2,  cm
  unsigned char  intentOne : 8 ;        //0~255
  unsigned char  intentTwo : 8 ;        //0~255
}SCDRFANS_POINT_S;

const int RFANS_LASER_COUNT = 16 ;
typedef struct {                                        //138 byte
  unsigned char           dataID : 8;                   //Sync Number
  unsigned char           chksum : 8;                   //chksum  Bytes[3,138]
  unsigned int            t0stampH  : 32;               //T0 Bloceek
  unsigned int            t0stampL  : 32;               //T0 tag
  SCDRFANS_POINT_S        laserData[RFANS_LASER_COUNT] ;//
}SCDRFANS_BLOCK_S;

static const int DECODE_BUFFER_SIZE = 0x40000; // 256KB

static const int UDP_PACKET_SIZE = 2*1024; // 256KB
static const int UDP_PACKET_BUFFER_SIZE = 16*1024; // 256KB
typedef struct  {
  int packetsize ;
  unsigned char buffer[UDP_PACKET_SIZE];
}UDP_PACKET_S;

typedef struct {
  int wrHead, rdTail, bufSize;
  UDP_PACKET_S buffer[UDP_PACKET_BUFFER_SIZE];
} UDP_PACKET_BUFFER_S;


typedef struct {
  int wrHead, rdTail, bufSize;
  unsigned char buffer[DECODE_BUFFER_SIZE];
} UDP_DECBUFFER_S;

typedef struct
{
  unsigned short range;
  unsigned char intensity;
}RFans_Laser32Block_S;

const unsigned short RFANS_UDPFRAMV6G_FLAT = 0xFFEE;
const unsigned short LASER32BLOCK_COUNT = 32;
typedef struct
{
  unsigned short flag; 
  unsigned short azimuthAngle;
  RFans_Laser32Block_S laserBlock[32];
}RFans_DataBlock_S;
const unsigned short UDP32FRAMEV6G_COUNT = 12;
const unsigned short RFANS_GM_16_FLAG = 0x3732;
const unsigned short RFANS_V6_GM_33_FLAG = 0x3733;


typedef struct
{
  SCDRFANS_BLOCK_S blockdata[10];
}RFans_UDPFRAMEV5_S;

typedef struct
{
  RFans_DataBlock_S dataBlock[UDP32FRAMEV6G_COUNT];
  unsigned int gpsTimestamp;
  unsigned char gmReservedA;
  unsigned char gmReservedB;
}RFans_UDP32FRAMEV6G_S;


typedef enum{
    LEVEL0_ECHO=0,
    LEVEL0_DUAL_ECHO,
    LEVEL1_ECHO,
    LEVEL1_DUAL_ECHO,
    LEVEL2_ECHO,
    LEVEL2_DUAL_ECHO,
    LEVEL3_ECHO,
    LEVEL3_DUAL_ECHO,

}MULTI_LEVEL_DATA;


typedef enum {
    DATA_LEVEL_ORI = 0,     
    DATA_LEVEL_CALIB,       
    DATA_LEVEL_USER,        
    DATA_LEVEL_USER_SIMPLE, 
} DATA_LEVEL_E;

// algorithm flag
typedef enum {
    ALGORITHM_ID_PREPROCESS = 0x00, //预处理模块
    ALGORITHM_ID_DISTANCE = 0x10,   //距离标定
    ALGORITHM_ID_INTENSITY = 0x10,  //灰度标定
    ALGORITHM_ID_ANGLE = 0x20,      //角度标定
    ALGORITHM_ID_STRETCHing = 0x30, //灰度拉伸
} ALGORITHM_ID_E;


// mirror id

typedef enum {
    MIRROR_ID_CFANS_00 = 0x0,   //CFANS ...
    MIRROR_ID_CFANS_01 = 0x1,
    MIRROR_ID_CFANS_02 = 0x2,
    MIRROR_ID_RFANS = 0x3,      //RFANS镜面标识
} MIRROR_ID_E;


// data packet id
typedef enum {
    PACK_ID_DUALECHO_MIX    = 0x00,   //双回波混合输出
    PACK_ID_DUALECHO        = 0x01,   //双回波输出
    PACK_ID_STRONG_ECHO     = 0x02,   //最强回波输出
    PACK_ID_FIRST_ECHO      = 0x03,   //第一回波输出

//    PACK_ID_DUAL_ECHO_MIX       = 0x01,
//    PACK_ID_TRIPLE_ECHO         = 0x0A,
//    PACK_ID_DUAL_ECHO           = 0x0C,
//    PACK_ID_SIGNLE_STRONG_ECHO  = 0x0D,
//    PACK_ID_SIGNLE_FIRST_ECHO   = 0x0E,
} PACK_ID_E;

static const int GROUP_NUM_ORI = 6;
static const int GROUP_NUM_CALIB = 6;
static const int GROUP_NUM_USER = 10;
static const int GROUP_NUM_USER_SIMPLE = 12;
static const int POINT_NUM_ORI = 32;
static const int POINT_NUM_CALIB = 32;
static const int POINT_NUM_USER = 16;
static const int POINT_NUM_USER_SIMPLE = 32;

// 0级数据封装：
typedef struct {
    unsigned short range;
    unsigned short rising_edge;
    uint8_t intensity_pulse[3]; //{Intensity[23:12]，Pulse Widt[11:0]}
} POINT_ORI_S;

typedef struct {
    // flag 定义
    //	2 bit 数据分级
    //	10bit 算法模块编号
    //	2 bit 镜面标识
    //	2 bit 数据打包格式编号
    unsigned short flag;
    unsigned short azimuth_angle;
    POINT_ORI_S points[POINT_NUM_ORI];
} GROUP_ORI_S;

typedef struct {
    GROUP_ORI_S groups[GROUP_NUM_ORI];
    uint8_t reserved[32];
    uint32_t gps_timestamp;
    unsigned char gmReservedA;
    unsigned char gmReservedB;
} PACKET_ORI_S;//1406 byte

//1级数据封装同0级数据
//2级数据封装：12bit intensity
typedef struct {
    uint16_t range1;
    uint16_t range2;
    uint8_t intents[3]; //{intensity1[23:12]，intensity2[11:0]}
} POINT_USER_S;           // contains 2 point

typedef struct {
    uint16_t flag;
    uint16_t azimuth_angle;
    POINT_USER_S points[POINT_NUM_USER];//16 points
} GROUP_USER_S;

typedef struct {
    GROUP_USER_S groups[GROUP_NUM_USER];
    uint8_t reserved[40];
    uint32_t gps_timestamp;
    unsigned char gmReservedA;
    unsigned char gmReservedB;
} PACKET_USER_S;//1206byte

//3级数据封装：
typedef struct {
    uint16_t range;
    uint8_t intensity;
} POINT_USER_SIMPLE_S;

typedef struct {
    uint16_t flag;
    uint16_t azimuth_angle;
    POINT_USER_SIMPLE_S points[POINT_NUM_USER_SIMPLE];
} GROUP_USER_SIMPLE_S;

typedef struct {
    GROUP_USER_SIMPLE_S groups[GROUP_NUM_USER_SIMPLE];
    uint32_t gps_timestamp;
    //uint16_t factory;
    unsigned char gmReservedA;
    unsigned char gmReservedB;
} PACKET_USER_SIMPLE_S;//1206 byte
#pragma pack()

#ifdef __cplusplus
extern "C"
{
#endif
int swapchar( unsigned char * _data, int size_ ) ;
int checkSum(unsigned char * _dataBuf, int count_ ) ;

DEB_FRAME_S packDEBV3Frame(SCD_FRAME_TYPE_E flag, int regAddress_, int regData_);

void writeFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, char * _mt_frame, int mt_size);

void readDEBFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, DEB_FRAME_S *mtRegMap);

int processFrameV5( RFans_UDPFRAMEV5_S *mtFrame,std::vector<SCDRFANS_BLOCK_S> &outBlocks);
int processFrameV6G(RFans_UDP32FRAMEV6G_S *mtFrame, std::vector<SCDRFANS_BLOCK_S> &outBlocks);
//int  searchUDPPacket(UDP_DECBUFFER_S *mtUdpBuffer, BLOCK_VECTOR_S *outBlocks);

int searchBlock(unsigned char *data, int size,int &flag,
								SCDRFANS_BLOCK_S *outBlock) ;

#ifdef __cplusplus
}
#endif


#endif
