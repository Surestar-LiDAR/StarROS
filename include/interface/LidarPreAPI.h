/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
/*     IMP文件解算功能使用步骤   
  
  1.配置解算参数(setup)
     
  2.开始解算(calcStart)

  3.读取解算进度(getProgress)

  4.停止解算(calcStop)
*/

/*     实时解算功能使用步骤             
  1.配置解算参数(setup)

  2.开始解算(calcRun)

*/



#ifndef _LIDARPREAPI_H
#define _LIDARPREAPI_H

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>

//#include <io.h>

#include "ICD_LiDAR_PRE.h"
#include "ICD_LiDAR_API.h"

#if defined(_MSC_VER)
#ifdef LIDARPRE_EXPORTS
#define LIDAR_PRE_DLL __declspec(dllexport)
#else //LIDARPRE_EXPORTS
#define LIDAR_PRE_DLL __declspec(dllimport)
#endif //LIDARPRE_EXPORTS
#else //defined(_MSC_VER)
#define LIDAR_PRE_DLL
#endif //defined(_MSC_VER)

class CLidarPre ;

class LIDAR_PRE_DLL CLidarPreAPI {
public:
  CLidarPreAPI( ) ;
  ~CLidarPreAPI() ;

  /**
  * @brief 解算参数配置
  * @_mt_preCtr ：解算参数
  * @mt_deviceTyp: 设备类型,例如(地面设备:eUArm)
  */
  bool setup(DEVICE_TYPE_E mt_deviceTyp, PRE_FILTER_S mt_prePara,PRE_WSG_S *mt_wsgPara=NULL ) ; 

  /**
  * @brief 文件解算进度返回
  * @_mt_process: 返回解算进度
  * @return ：
  */
  bool getProgress(FileProcess_S *_mt_process );

  /**
  * @brief IMP文件解算 
  * @mt_rawFiles_ ：IMP文件列表,按文件采集时间顺序排列
  * @_mt_strOutPath ：   解算输出目录
  * @return :  false: 文件个数为零。
  */
  bool calcStart( std::vector<std::string> mt_rawFiles_, char *_mt_strOutPath, PRE_WSG_S *_mt_wsgPara) ;	

  bool calcStop() ;

  /**
  * @brief 实时解算
  * @_mt_headInfo ：头信息。 HeadSize = 80KB
  * @_mt_fastFlow ：快数据流地址
  * @mt_flowSize  :  快数据流大小，512KB >= mt_flowSize
  * @_mt_wsgPara  :  大地坐标参数 value = NULL 设备坐标解算,反之大地坐标解算。
  * @__outResult  :  输出解算数据
  * @return：
  */
  
  bool calcRun(PRE_BUFFER_S *_mt_fastFlw, PRE_COORD_S *_outResult, PRE_BUFFER_S *_mt_headInfo , PRE_WSG_S *_mt_wsgPara) ;	
  
private:
  
};

#ifdef __cplusplus
extern "C"
{
#endif
  LIDAR_PRE_DLL bool CSetup(DEVICE_TYPE_E mt_deviceTyp, PRE_FILTER_S mtPrePara,PRE_WSG_S *mtWsgPara=NULL) ;
  LIDAR_PRE_DLL bool CCalcStart( std::vector<std::string> mt_rawFiles_, char * _mt_strOutPath, PRE_WSG_S *_mt_wsgPara) ;
  LIDAR_PRE_DLL bool CCalcStop() ;
  LIDAR_PRE_DLL bool CCalcRun( PRE_BUFFER_S *_mt_fastFlw, PRE_COORD_S *_outResult, PRE_BUFFER_S *_mt_headInfo , PRE_WSG_S *_mt_wsgPara) ;
  LIDAR_PRE_DLL bool CGetProgress(FileProcess_S *_mt_process) ;
  LIDAR_PRE_DLL void CInitPre() ;
  LIDAR_PRE_DLL void CClearPre() ;
  LIDAR_PRE_DLL void CDecodeProcess() ;
  LIDAR_PRE_DLL void CCalcProcess() ;
#ifdef __cplusplus
}
#endif


#endif
