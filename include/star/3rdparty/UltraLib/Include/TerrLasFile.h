/********************************************************************
       Author:		           ZQW	
       File Name:	           TerrLasFile
	   File Type:	           h
       Created Date:	       2011/8/28
	   Created Time:	       15:11      
*********************************************************************/
#ifndef TerrLasFile_h_ZQW_2011_8_28_15_11_DEF
#define TerrLasFile_h_ZQW_2011_8_28_15_11_DEF

#include "FileAnly.hpp"
#include "TerrPhotoFile.h"

#ifndef TerrLasFile_LIB 
   #define TerrLasFile_LIB _declspec(dllimport) 
   #ifdef _DEBUG
       #pragma comment(lib,"TerrLasFileD.lib")
       #pragma message("Automatically linking with TerrLasFileD.lib")
   #else
       #pragma comment(lib,"TerrLasFile.lib")
       #pragma message("Automatically linking with TerrLasFile.lib")
   #endif
#endif

#ifndef  _LASERPARA
#define  _LASERPARA

#define  MAXLASSTRIP  256
#define  MAXLASCLASS  18
enum eClassType
{ 
	eNeverClassified=0,
	eUnclassified=1,
	eGround=2,
	eLowVegetation=3,
	eMediumVegetation=4,
	eHighVegetation=5,
	eBuilding=6,
	eNoise=7,
	eModelKey=8,
	eWater=9,
	eBridge=10,
	eRoad=11,
	eOverlap=12,
	eRailroad=13,
	eStream=14,
	ePowerLine=15,
	ePowerTower=16,
	eRemoved=17,
};
//Header for LaserFile;
typedef struct tagLASERHDR{
	char           strVersion[64];
	double         dOffX,dOffY,dOffZ;
	double         minX,minY,minZ;   
	double         maxX,maxY,maxZ;   
	int            nUnitByteSize;
	int            nUnitNum;
	int            nTolReturn;
	int            nMaxAngle;
	int            nMaxClass;
	int            nMaxIntens;
	int            nMinIntens;
}LASERHDR,*PLASERHDR;

typedef struct tagLASERNODE{	
	float          x,y,z;
	unsigned short point_source_ID;	
	unsigned short Intensity;	
	unsigned char  Return: 3;
	unsigned char  ReturnNumber : 3;	
	unsigned char  scan_direction_flag :1;
	unsigned char  edge_flight_flag :1;
	unsigned char  red;
	unsigned char  green;
	unsigned char  blue;	
	unsigned char  Class : 7;
	unsigned char  bSkip : 1;
	short  ScanAngle;
}LASERNODE,*PLASERNODE;

typedef struct tagPOINTNODE{	
	float          x,y,z;	
	unsigned char  Class;	
	unsigned char  Return : 4;
	unsigned char  ReturnNumber : 4; 
	unsigned short point_source_ID;
}POINTNODE,*PPOINTNODE;

//for Gridline;
typedef struct tagRECTBLOCK
{ 	
	char    strID[IDLEN];
	int     nFlag;	
	double  MinX,MaxX;
	double  MinY,MaxY;
}RECTBLOCK,*PRECTBLOCK;
typedef struct tagCUBEBLOCK
{ 	
	char    strID[IDLEN];
	int     nFlag;	
	double  MinX,MaxX;
	double  MinY,MaxY;
	double  MinZ,MaxZ;
}CUBEBLOCK,*PCUBEBLOCK;
#endif

class TerrLasFile_LIB CEditOper
{
public:	
    #define   MAXUNDO    50
	typedef struct tagBUFMODE{
		LASERNODE *pNode;
		int        nType;
	}BUFNODE,*PBUFNODE;

public:
	void Reserve(LASERNODE** ppBuflst,int nBufSize,BOOL bClean=FALSE);
	void Redo();
	void Undo();
	BOOL CanRedo();
	BOOL CanUndo();
	void Clean();
	CEditOper();
	virtual ~CEditOper();

public:		
	char m_strTmp[PATHLEN];
	int  m_nCount,m_nCurt;
};

class TerrLasFile_LIB CLaserFile
{
public:		
	enum ePointType
	{ 
		ePoint=0,
		ePointGPS=1,
		ePointRGB=2,
		ePointGPSRGB=3,
	};

public:
	//¶ÁÐ´²ÎÊý£»
	static BOOL        LoadConfig(const char* lpstrPath,int *nSkip,int *classFlag);
	static BOOL        SaveConfig(const char* lpstrPath,int nSkip,int *classFlag);	

	static BOOL        TransformLas(const char* lpstrInLas,const char* lpstrOutLas,double *p7Par,double Coord_Scale=0.001,HWND hMsgWnd=NULL);
	static BOOL        WriteStationFlag(const char* lpstrInLas,const char* lpstrOutLas,int nStation,HWND hMsgWnd=NULL);
	static BOOL        WriteLaserFile(const char* lpstrOutLas,LASERNODE **ppCldLst,int nCldNum,double xOff,double yOff,double zOff,int *Class,double Coord_Scale=0.001,HWND hMsgWnd=NULL);
	static BOOL        WriteLaserFile(const char* lpstrOutLas,LASERNODE  *pCldLst ,int nCldNum,double xOff,double yOff,double zOff,int *Class,double Coord_Scale=0.001,HWND hMsgWnd=NULL);
	static void        StatisticHdr4List(PLASERHDR pLasHdr,PLASERNODE   pLasLst,int nPtNum,double dOffX=0.0,double dOffY=0.0,double dOffZ=0.0);
	static void        StatisticHdr4List(PLASERHDR pLasHdr,PLASERNODE* ppLasLst,int nPtNum,double dOffX=0.0,double dOffY=0.0,double dOffZ=0.0);	
	static BOOL        GetLasRange(vector<string>& vFileList,double *xMin,double *yMin,double *zMin,double *xMax,double *yMax,double *zMax);
	static BOOL        GetTimeRange(const char* lpstrFilePath,double *stTime,double *edTime);
	static BOOL        GetLaserOffset(const char* lpstrFilePath,double *xOffset,double *yOffset,double *zOffset);
	static void        GetZRange(LASERNODE **ppCldLst,int nLstSize,int *Class,float *zMin,float *zMax);
	static BOOL        GetStationNO(const char* lpstrFilePath,int *nStation);
	static BOOL        ReadHdr4File(const char* lpstrFilePath,PLASERHDR pLasHdr);
	static BOOL        ReadLaserFile (const char* lpstrFilePath,PLASERHDR pLasHdr,PLASERNODE pCldLst,double xOff,double yOff,double zOff,int *Class,int nSkip=1);
	static BOOL        ReadPoint4File(const char* lpstrFilePath,PLASERHDR pLasHdr,PPOINTNODE pSmartLst,double xOffset,double yOffset,double zOffset,int *Class,int nSkip=1);
	static void        InitAllClass(int *Class,int nValid=1);
	static void        UpdateSkip4Class(int *pFlag,LASERNODE **ppCldLst,int nLstSize);
	static void        UpdateSkip4SrcID(int *pFlag,LASERNODE **ppCldLst,int nLstSize);

public:	
	BOOL               LoadLasFile(vector<string>& vStrlist,PRECTBLOCK pRtRange,double *Offset,int *Class,int nSkip=1);
	BOOL               LoadLasFile(const char* lpstrFilePath,double xOffset,double yOffset,double zOffset,int *Class,int nSkip=1);
	BOOL               SaveLasFile(const char* lpstrFilePath,int *Class,int nPointType=ePointGPSRGB,double Coord_Scale=0.001);	
    BOOL               IsLoaded();

	BOOL               Attach(PLASERHDR pLasHdr,LASERNODE **ppLasLst);
	void               Detachs();

	LASERHDR*          GetLasHdr(){return (&m_CldHdr);};	                               
	LASERNODE**        GetLasLst(){return m_ppCldLst;};
	const char*        GetLasPath(){return m_strPath;};
	void               SetTieSize(int nSize=1000000){m_nTieSize=nSize;}

	void               SetMsgWnd(HWND hMsgWnd=NULL){m_hMsgWnd=hMsgWnd;};
	void               ProcMsg(DWORD dwMsgID,DWORD dwMsgType,LPARAM lpParam=0){if(::IsWindow(m_hMsgWnd)){::SendMessage(m_hMsgWnd,dwMsgID,dwMsgType,lpParam);};};

	CLaserFile();
	virtual ~CLaserFile();	

protected:	
	char               m_strPath[PATHLEN];
	LASERHDR           m_CldHdr;
	LASERNODE        **m_ppCldLst;
	vector<PLASERNODE> m_vCldLst;
	int                m_nTieSize;
	HWND               m_hMsgWnd;

private:
	void           Clear();	
};

class TerrLasFile_LIB CLasMgr
{
public:				
	BOOL                AddLaserFile(const char* lpstrPathFile,int *nClassFlag,double xOff,double yOff,double zOff,int nSkip=1,BOOL bUpdate=TRUE);
	BOOL                SaveLaserList(int *Class);
	BOOL                SaveLaserListAs(const char* lpstrPathFile,int *Class);
	void                Clear();
	
	BOOL                CutPolygon(POLYGON *pPolygon,int *Class,const char* lpstrOutPath,double Coord_Scale=0.001);
	int                 GetLasNum(){return (m_vLasList.size());};
	LASERHDR*           GetCldHdr(){return (&m_CldHdr);};
	LASERNODE**         GetCldLst(){return m_ppCldLst;};

	void                SetMsgWnd(HWND hMsgWnd=NULL){m_hMsgWnd=hMsgWnd;};
	void                ProcMsg(DWORD dwMsgID,DWORD dwMsgType,LPARAM lpParam=0){if(::IsWindow(m_hMsgWnd)){::SendMessage(m_hMsgWnd,dwMsgID,dwMsgType,lpParam);};};

	CLasMgr();
	virtual ~CLasMgr();

public: 
	vector<CLaserFile*> m_vLasList;
	vector<int>         m_vSourceID;
	LASERNODE         **m_ppCldLst;
	LASERHDR            m_CldHdr;
	HWND                m_hMsgWnd;	
};

class TerrLasFile_LIB CLasSpliter
{
public:	
	BOOL                SplitQuad(vector<string>& vLasPath,const char* lpBlkDir,int nMaxBlkNum);	
	BOOL                SplitOct(vector<string>& vLasPath,const char* lpBlkDir,int nMaxBlkNum);	
	BOOL                SplitBgn(vector<string>& vLasPath,const char* lpBlkDir,const char* lpBgnPath);

	void                SetMsgWnd(HWND hMsgWnd=NULL){m_hMsgWnd=hMsgWnd;};
	void                ProcMsg(DWORD dwMsgID,DWORD dwMsgType,LPARAM lpParam=0){if(::IsWindow(m_hMsgWnd)){::SendMessage(m_hMsgWnd,dwMsgID,dwMsgType,lpParam);};};
	
	CLasSpliter();
	virtual ~CLasSpliter();

protected:
	vector<string>      m_vLasList;
	HWND                m_hMsgWnd;

private:
};
class TerrLasFile_LIB CGridMgr
{
public:	
	enum eUnitType{eNormal=0,eSelected=1,eDeleted=2,};

	BOOL          Init(double xStart,double yStart,double xEnd,double yEnd,double xInterval,double yInterval);
	BOOL          LoadBgn4File(const char* lpstrBgnPath);
	BOOL          SaveBgn2File(const char* lpstrBgnPath);	

	BOOL          LabelbyPoints(double xOffset,double yOffset,LASERNODE **ppCldLst,int nCldNum);
	void          SelectAreaRects(PRECTBLOCK pRtBox);
	PRECTBLOCK    PickRect(double x,double y);

	int           GetValidPath(const char* lpstrDir,char* pPathLst,int nType=eSelected);
	int           GetValidNum(int nType=eSelected);
	int           GetSelectedNum();
	int           GetDeletedNum();
	void          SetRectColor(int nUnitType,float fRed,float fGreen,float fBlue);

	CGridMgr();
	virtual ~CGridMgr();
public:
	double        m_xInterval,m_yInterval;
	double        m_xStart,m_yStart,m_xEnd,m_yEnd;	
	PRECTBLOCK    m_pRectLst;
	PT3F          m_ColorMap[5];
	int           m_nCols,m_nRows;
private:
	void          UpdateRectID();
};
class TerrLasFile_LIB CLasFusion
{
public:
	BOOL                Initialize(const char* lpstrCamPath,const char* lpstrList,int nBuffSize=3000000);
	BOOL                ExecFusion(const char* lpstrLasPath,const char* lpstrFusionPath);

	void                SetMsgWnd(HWND hMsgWnd=NULL){m_hMsgWnd=hMsgWnd;};
	void                ProcMsg(DWORD dwMsgID,DWORD dwMsgType,LPARAM lpParam=0){if(::IsWindow(m_hMsgWnd)){::SendMessage(m_hMsgWnd,dwMsgID,dwMsgType,lpParam);};};

	CLasFusion();
	virtual ~CLasFusion();

protected:
	float              *m_pPx,*m_pPy;
	int                *m_pIndex;
	double             *m_pRotlist;
	CCameraFile         m_camFile;
	CTerrImgList        m_imgList;

private:
	BOOL                resizeBuffer(int nBufSize);

	PLASERNODE          m_pBuffer;
	int                 m_nBufSize;
	HWND                m_hMsgWnd;
};



#endif // TerrLasFile_h__