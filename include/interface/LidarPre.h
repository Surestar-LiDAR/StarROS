/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
#ifndef _LIDAR_PREC_H
#define _LIDAR_PREC_H
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <list>
#include "ICD_LiDAR_PRE.h"
#include <istoolkit/impHeader.h>
#include "ICD_LiPRE_API.h"

using namespace std;

#ifdef _WINDOWS
#include <atlstr.h>
#endif

#define  UTCDIFF  1
#pragma pack(2)
typedef struct _SOutMsg{
  FILE *fp;
  FILE *z_fp;
  bool isOut;
  _SOutMsg(){
    fp=NULL;
    z_fp=NULL;
    isOut=false;
  }
}OutMsg_S;

typedef struct _SOutFile{
  OutMsg_S plr;
  OutMsg_S bplr;
  OutMsg_S xyzi;
  OutMsg_S bxyzi;
  OutMsg_S las;
  OutMsg_S ptx;
  OutMsg_S ysj;
  OutMsg_S st;
  OutMsg_S crs;  
  OutMsg_S agr ;
  OutMsg_S bagr ;
  OutMsg_S cpt;
  OutMsg_S grid;
OutMsg_S rfans ;
OutMsg_S rgp ;
}OutFile_S;

typedef struct _SPathPara {
  char taskPath[256] ;
  std::vector<std::string> impVector ;	///<原始数据路径
  char outPath[256] ;
  char algPath[256] ;
  char sbetPath[256] ;
  char currFile[256] ;
  _SPathPara() {
    memset(taskPath,'\0',256);
    impVector.clear() ;
    memset(outPath,'\0',256);
    memset(algPath,'\0',256);
    memset(sbetPath,'\0',256);
    memset(currFile,'\0',256);
  }
} PathPara_S ;

typedef struct _SFileNum {
  int imp_totalnum;
  int imp_surplusnum;
  _SFileNum() {
    imp_totalnum=0;
    imp_surplusnum=0;
  }
}FileNum_S;

#pragma pack()

class AlgManager;
class ImpWorkFlow;
class CimpDecode;
class Cinterpolation;
class CLidarConfig;
class COutPutLas;
class COutPutPLR;
class COutputXYZI;
class COutputPTX;
class COutPutYSJ;
class IntAlgorithm;
class COutputST ;
class COutputCRS ;
class COutputCPT ;
class COutputRGP ;
class COutPutAGR ;
class CMeteoricCorrection;
class CRangeAngCorrection ;
class COutputRfans ;
class COutputGrid ;

typedef struct _Preptr {
    Cinterpolation * _interp;
    ImpWorkFlow * _wflow;
    AlgManager * _alg;
    COutPutPLR * _outplr;
    COutputXYZI * _outxyzi;
    COutPutYSJ * _outysj;
    COutputCRS * _outcrs ;
     COutputCPT * _outcpt ;
     COutputRGP * _outrgp ;
     COutputGrid * _outGrid ;
} _Preptr_S ;

typedef struct _ID_Time{
  unsigned int ID;
  unsigned int Stamp;
  double Utc;
  char filename[256];
}ID_Time_S;

/*typedef struct _PosEvent_Camera{
  vector<ID_Time_S>  tmp_posEvent,tmp_camera ,tmp_ppsPair;   //从文件读取的数据 / 添加插入点的数据（如合并raw_posEvent与updata_posEvent）
  vector<ID_Time_S>  raw_posEvent,raw_camera ;             //插入点之前的数据
  vector<ID_Time_S>  updata_posEvent,updata_camera ;    //插入点和插入点之后的数据
}PosEvent_Camera_S;*/

#ifndef LIDAR_PRE_DLL

#if defined(_MSC_VER)

#ifdef LIDARPRE_EXPORTS
#define LIDAR_PRE_DLL __declspec(dllexport)
#else //LIDARPRE_EXPORTS
#define LIDAR_PRE_DLL __declspec(dllimport)
#endif //LIDARPRE_EXPORTS

#else //LIDAR_PRE_DLL
#define LIDAR_PRE_DLL
#endif //defined(_MSC_VER)

#endif //LIDAR_PRE_DLL


class LIDAR_PRE_DLL CLidarPre {
 public: 
  CLidarPre(SHOTS_RINGBUF_S * shotsc = NULL) ;
  ~CLidarPre() ;
  static CLidarPre *getInstance() ;
  static void destroyInstance() ;
  int getDevMacroDefine();
  //void checkT0Utc(FILE *posEvent_file,FILE * camera_fil,FILE *ppsPair_fil,FILE *fil_out) ;

 void checkPosCameraEvent(FILE *posEvent_file,FILE * camera_fil,FILE *ppsPair_fil,FILE *fil_out) ;
 void findCameraUtc(FILE *lidar_fil , FILE *camera_fil,FILE *fil_out) ;

  CLidarConfig *getLidarCfg();

  int setPrePara(const char *cfgfile,PRE_CONTROL_S * pctrl);   //将setPreConfig和setPreControl合并

  int setOutOpt(ALGSC_ARRAY_S *modpara);

  int setInput(std::vector<std::string>fileA);

  void clearImpFileVector() ;
  int addImpFile(const char * impfile);

  void setOutput(const char * strpath) ;
  char * getOutput() ;

  void setPos(char *strpath) ;
  char * getPos(){return m_dataPath.sbetPath;}
  //获取解算相关模块的指针
  _Preptr_S getPrePtx();

  int setWorkflow(std::list <int>&ProModule,WORK_FLOW_E eWftyp=eUrRealTm);

  int run(bool inRing, IMP_DCOOPT_E eopt) ;
  
  char * getAlgName(int algID);
  char * getAlgDescripts(int algID);
  int getAlgId(int algID);
  FileProcess_S getCalVelocity(); 

  CALCOUT_BUFFER_S * getCalcBuffer() ;
  //实时解码
  PRE_POINTS_S * getDecodeBuffer(char *_mt_imphead, int mt_hdSize,char *_mt_dataBuff, int mt_bfSize) ;

  STAT_DIS_S *getStatDis() ;
  std::vector<std::string> getImpVector() ;
  
  std::vector<std::string> common_Proc(std::vector<std::string> rawFolderPath);
  bool common_Proc(char *_mt_FolderPath , LiPRE_DIRTYPE_E mt_dirType = eIMPDIR ) ;
  void handleDataFile(char *DirSpec,string dataPath);
  void handleDataFile(char *_mt_dirSpec ) ;
  FileNum_S getImpNum();
  void slowFileSizeSetup(unsigned int mt_size ) ;

  //大气校正距离
  Meteotic_s getMeteCorr(bool model,double Wavelen,double Temperature,double Pressure,double DewTemperature);
  Meteotic_s gettmpa(double PlatformHeight,double GroundTM);
  double getintegralcof(bool model,double Wavelen, double DewTemperature,double PlatformHeight,double GroundTM) ;
  
  //解算参数配置
  bool cfgPreParameter(PRE_CONTROL_S * _mt_preCtr ,DEVICE_TYPE_E mt_deviceTyp, int mtMode = 4) ;

  /**************************/
  /*   实时解算功能接口定义 */
  /**************************/
  bool loadImpHead(char * _mt_headInfo,int size) ;

  PRE_POINTS_S * flowDecode(char * _mt_fastFlw, int mt_flwSize ) ;
  PRE_COORD_S * deviceCoordinate(char * _mt_fastFlw, int mt_flwSize, char * _mt_slowFlw, int mt_slwSize) ;
  PRE_COORD_S * geoprojectCoordinate(char * _mt_fastFlow, int mt_flowSize, SBET_Rec * _sbetbuffer, int mt_sbetCount ) ;

  bool setup(DEVICE_TYPE_E mt_deviceTyp, PRE_FILTER_S mt_prePara, PRE_WSG_S *mt_wsgPara) ;
  bool calcStart( std::vector<std::string> mt_rawFiles_, char * _mt_strOutPath, PRE_WSG_S *_mt_wsgPara) ;
  bool calcStop() ;
  bool calcRun( PRE_BUFFER_S *_mt_fastFlw, PRE_COORD_S *_outResult, PRE_BUFFER_S *_mt_headInfo , PRE_WSG_S *_mt_wsgPara) ;
  bool decodeProcess() ;
  bool calcProcess() ;
  bool setProcess(bool mtFlag) ;

  PRE_COORD_S* outPreBuf() { return &m_preCoord_; };
  int bCptFileToTextCptFile(char * _mt_fileFullName) ;
  IMP_DCORING_S * getDecodeRing() { return &m_dcoRing_; }
  void setShotRingBuffer(SHOTS_RINGBUF_S * mtRing);
  int writeCalcBuffer(SHOTS_CALCOUT_S mtPoint);
private: 
  vector<ID_Time_S>  m_posEvent_Date,m_cameraEvent_Date ,m_ppsPair_Date;   //从文件读取的数据 / 添加插入点的数据（如合并raw_posEvent与updata_posEvent）
  vector<ID_Time_S>  m_posEvent_Pre,m_camera_Pre ;             //插入点之前的数据
  vector<ID_Time_S>  m_posEvent_Last,m_camera_Last ;    //插入点和插入点之后的数据

 vector<ID_Time_S> m_lidar_Date, m_camera_Date;
 // PosEvent_Camera_S m_posEvent_Camera;
  //SHOTS_CALCOUT_S _calcBuffer[SHOT_RINGBUF_SIZE] ;
  CALCOUT_BUFFER_S m_calcBuffer ;  //解算输出结果

  PRE_COORD_S m_preCoord_ ; 
   
  STAT_DIS_S _stat_dis;

  CLidarConfig * _lidarConfig ;
  AlgManager * _algManager ;
  Cinterpolation * _interpolate ;
  CimpDecode * _impDecoder ;
  ImpWorkFlow * _impWorkflow;
  CMeteoricCorrection * _meteCorr; //大气校正距离
  CRangeAngCorrection *_m_rangeAngCorr ;
  
  //
  
  SHOTS_RINGBUF_S * _rawShotRing ; // shotsCloud ;
  
  IMP_DCORING_S m_dcoRing_ ; //解码输出结果

  //文件头
  Imp_Header_S m_impheader ;
  bool m_binRing  ;
  IMP_DCOOPT_E m_eOutput ;

  std::vector<std::string> m_impVector ;	///<原始数据路径
  int m_total_imp;

  bool iscloseimp;

  PathPara_S m_dataPath ; //zks: no logic

  OutFile_S m_outfile;
  COutPutLas *m_outlas;	  //las文件输出类对象
  COutPutPLR *m_outplr;   //plr文件输出类对象
  COutputXYZI *m_outxyzi; //xyzi文件输出类对象
  COutputPTX *m_outptx;   //ptx文件输出类对象
  COutPutYSJ *m_outysj;   //ysj文件输出类对象
  COutputST * _m_outst ;  //st 分镜面文件输出对象
  COutputCRS * _m_outcrs ;   //crs 文件输出对象
  COutputCPT * _m_outcpt ;   //cpt 文件输出对象
  COutputRGP * _m_outrgp ;   //cpt 文件输出对象
  COutPutAGR * _m_outarg ;   //agr 文件输出对象
   COutputRfans * _m_outrfans ;  
   COutputGrid * _m_outgrid;   //grid 文件输出对象
  char _calcFullPath[256];
  char newraw[256];

  unsigned int filesize_las;
  FileProcess_S fileprocess;

  unsigned int scanLineNum[2];
  unsigned int linePointNum;
  unsigned int writeIndex[2];

  _Preptr_S m_PrePtr;
  PRE_POINTS_S m_prePoints_ ;//实时解码激光点

  bool  read_nextfileSlow;   //是否读取下一个imp的慢数据
  bool m_eraseImp;    
  char m_impHeadBuf[IMP_HEADER_SIZE] ; //IMP head info
  PRE_CONTROL_S m_setupPara_ ;
  DEVICE_TYPE_E m_deviceType ;
  int m_preFileCount ;
  bool m_threadRun ;

  bool m_isSlowDataSplit;//慢数据拆分
  bool m_ischeckUSC;//是否验证加密
  bool m_isgettaildatasucc;
  char taildata[256];
  typedef enum {
	  e_NONE,
	  e_SUCCESS,
	  e_FAILED,

  }APPROVE; 
  APPROVE m_approve;
private:
  void checkFirstData(FILE *fil_out);     //检查posCamera_Event的第一个点
  void checkMediumTailData(FILE *fil_out);    //从第二个点检查posCamera_Event的
//插入数据后，更新posEvent和Camera数据
  void updataCameraDate(ID_Time_S cameraDate,int cameraNum,int position) ;
  void updataPosEventDate(ID_Time_S posDate,int PosNum,int position)  ;
  //初始化Ringbuffer
  void initRingB();
  void outFile(char * newfilename);
  void closeFilePointer();
  void closePtxFilePointer();
  void initFilePointer(char * newfilename);
  void initPtxFilePointer(char * newfilename);
  void create_file(char * tmpfname);
  char *getNextImpFile();
  void creat_newfile(void);
  int reset();
  int lidarPreCalc() ; //!< 解码、解算过程
  void joinPTXFile(FILE *fp,FILE *z_fp) ;
  void decodeAnalysis(IMP_SHOTRING_S * mtshotRing, PRE_POINTS_S * mtOutBuffer);
  bool checkUSC(char*name = NULL);
  void resetCheckUSC();
  bool getTailData(char outdata[], char* filename,int len);
public:
  bool getApproveState(){return m_approve;}
  void setSlowFileOpened(bool isopened);

};
#endif
