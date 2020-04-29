/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
#ifndef ICD_LIDAR_TASK_H
#define ICD_LIDAR_TASK_H


#include "ICD_LiDAR_API.h"

//线程优先级
const int THRD_PRI_DEF    = 0 ; //lowest
const int THRD_PRI_TIMER = THRD_PRI_DEF + 0 ;  //计时器优先级
const int THRD_PRI_MSGF   = THRD_PRI_DEF+0  ; //Engine.svc 100
const int THRD_PRI_CMDF   = THRD_PRI_DEF+0  ; //cmd server (tcp server) 100
const int THRD_PRI_DATAF  = THRD_PRI_DEF+1 ; //Scada.svc 30
const int THRD_PRI_DATAH  = THRD_PRI_DEF+2 ; //data handle svc 30
const int THRD_PRI_DCDR	  = THRD_PRI_DEF+1 ; //data decode 30
const int THRD_PRI_CALC   = THRD_PRI_DEF+1 ; //data calc 30

static const int FILE_PATH_LEN = 1024; // Compatible with the POSIX standard
static const int FILE_NAME_LEN = 1024; // file name exclude file path



/************ for task files .tskx ************/

#pragma pack(2)

typedef struct {
  float m_taskver ;
  int m_timestamp ;
  char m_taskid[64] ;
} LDRTASK_HEAD_S ;

//1. zks: really required ???
const int UARM_HANGLE_MIN = 0;  //degree
const int UARM_HANGLE_MAX = 360;
const int UARM_VANGLE_MIN = 0;
const int UARM_VANGLE_MAX = 180;

const int ROIH_ANGLE_START_ = 0;
const int ROIH_ANGLE_STOP = 360;
const int ROIV_ANGLE_START = 0;
const int ROIV_ANGLE_STOP = 180;
const float ROIA_ANGLE_RES = 0.01f; //rad ???

const int  LiDAR_RANGE_MAX = 600 ;     //!< 最大测距默认值：600m
const int  LiDAR_RANGE_MIN = 1 ;        //!< 最小测距默认值：1 m

/****zks:  Replaced with the defination below
typedef struct { 
  // user defined
  float hAngStr, hAngStp ;
  float vAngStr, vAngStp ;
  int rangeMin, rangeMax ;
  // calculated
  float hSpcRes, vSpcRes ; // mm
  float hAngRes, vAngRes ; // degree
  int lsrFreq , scanFreq ;
  float stageSpeed ; // degree/sec ; or meter/sec
  // statistic
  int lineNum, colmNum, shotSum ;
  int duration ; // in seconds
  // cooperated camera actions
  //...
} LiDAR_ROIDEF_S ;
************************************************/

typedef struct { 
  // user defined
  float hAngStr, hAngStp, vAngStr, vAngStp ; //degree
} LiDAR_Region_S ;

typedef struct { //Grid Resolution
  float refRange ; //! 基准距离 , meter
  float hgres, vgres ; //! centimeter liyp
} LiDAR_RoiGrdRes_S ; 

typedef struct {//angle resolution
  float hares, vares ; // degree
} LiDAR_RoiAngRes_S ;

typedef struct {
  ECHO_TYPE_E m_echo ;
  int rangeMin, rangeMax ; // meters
  int lsrFreq ; // kHz
  int scnSpeed ; //RPM
  float stgSpeed ; // degree/sec 
  // cooperated camera actions
  //...
} LiDAR_ProgSet_S ;

typedef struct { 
  LiDAR_ProgSet_S progS ; 
  LiDAR_Region_S roi[4] ;  
  unsigned short roiNum  ;
} LiDAR_RoiProg_S ;

typedef struct {  // statistic
  int lineNum, colmNum, shotSum ;
  int duration ; // in seconds
} LiDAR_RoiCal_S ;

typedef struct { 
  // in 
  LiDAR_Region_S region;
  LiDAR_RoiAngRes_S angRes;

  // in/out
  LiDAR_ProgSet_S progSet;

  // out
  LiDAR_RoiGrdRes_S grdRes;
  LiDAR_RoiCal_S    roiCal;
} LiDAR_ROIDEF_S ;


typedef struct {
  int lsrFreqMin ;
  int lsrFreqMax ;
  int rangeMin ;
  int rangeMax ;
  int mpiaMode; 
}LiDAR_PROGRANGE_S;

#pragma pack()
  
#endif
