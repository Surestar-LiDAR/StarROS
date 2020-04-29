/********************************************************************
       Author:		           ZQW	
       File Name:	           TerrPhotoFile
	   File Type:	           h
       Created Date:	       2013/11/18
	   Created Time:	       15:10      
*********************************************************************/
#ifndef TerrPhotoFile_h_ZQW_2013_11_18_15_10_DEF
#define TerrPhotoFile_h_ZQW_2013_11_18_15_10_DEF

#include "BasicDef.h"

#ifndef TerrPhotoFile_LIB 
   #define TerrPhotoFile_LIB _declspec(dllimport)
   #ifdef _DEBUG
      #pragma comment(lib,"TerrPhotoFileD.lib")
      #pragma message("Automatically linking with TerrPhotoFileD.lib")
   #else
      #pragma comment(lib,"TerrPhotoFile.lib")
      #pragma message("Automatically linking with TerrPhotoFile.lib")
   #endif
#endif

#ifndef _CAMPARA
#define _CAMPARA
typedef struct tagCAMNODE{	
	char    strName[NAMELEN]; //相机名称；
	int     cols,rows;	 //面阵大小，单位像素；
	double  x0,y0;       //主点，单位mm；
	double  focal;	     //焦距，单位mm；
	double  pixSize;	 //像素大小，单位mm；
	double  k1,k2,k3,k4; //径向畸变；
	double  p1,p2;       //偏心畸变；
	double  b1;          //像素非正方形比例因子,s1,为薄棱镜畸变；
	double  b2;          //CCD阵列排列非正交性畸变系数,s2,为薄棱镜畸变；
}CAMNODE,*PCAMNODE;
#endif

#ifndef _PHOTONODE
#define _PHOTONODE
typedef struct tagPHOTONODE
{ 
	char   strFile[PATHLEN];	
	CAMNODE camera;
	double time;
	double xs,ys,zs;
	double roll,pitch,heading;	
	int    nIndex;
	int    nFlag;
}PHOTONODE,*PPHOTONODE;
#endif

#ifndef _TERRSERES
#define _TERRSERES
//for terrestrial tie point;
enum eRsgFlag{ePtTie=0,ePtImgTie=1,eImgTie=2,eInValidTie=3,};
typedef struct tagRGSTIENODE
{
	char         strTieID[IDLEN];	
	char         strImageName[NAMELEN];
	int          nStation;
	int          nFlag;
	double       px,py;
	double       ox,oy,oz;
	double       rpx,rpy;
	double       rox,roy,roz;	
}RGSTIENODE,*PRGSTIENODE;
typedef struct tagRGSPAIR
{
	int          nLeftStation;
	int          nRightStation;
	double       Transform[7];//From L to R;
}RGSPAIR,*PRGSPAIR;
//for terrestrial lidar image list;
typedef struct tagRGSIMGLIST
{
	double       Angle;
	char         strPath[PATHLEN];
	double       Transform[6];
}RGSIMGLIST,*PRGSIMGLIST;
//for terrestrial transform;
typedef struct tagLASERORI
{
	int          nStation;
	double       Transform[7];
	double       RotMx[9];
}LASERORI,*PLASERORI;
#endif

#ifndef _KPTNODE
#define _KPTNODE
typedef struct tagKPTNODE 
{	
	char         strID[IDLEN];	
	double       x,y,z;	
	short        horz;
	short        vert;	
}KPTNODE,*PKPTNODE;
#endif

class TerrPhotoFile_LIB CCameraFile
{
public:
	static void      DistorPix2mm(double *k1,double *k2,double *k3,double *k4,double *p1,double *p2,double *b1,double *b2,double pixSize);
	static void      Distormm2Pix(double *k1,double *k2,double *k3,double *k4,double *p1,double *p2,double *b1,double *b2,double pixSize);
	static void      Prip2Pix(const PCAMNODE pCamera,double mx,double my,double* px,double* py);
	static void      Pix2Prip(const PCAMNODE pCamera,double px,double py,double* mx,double* my);	
	
	BOOL             LoadCam4File(const char* lpstrPath);
	BOOL             SaveCam2File(const char* lpstrPath);
	
	BOOL             LoadCam4Lib(const char* lpstrPath);
	BOOL             SaveCam2Lib(const char* lpstrPath);

	CCameraFile();
	virtual ~CCameraFile();

public:
	CAMNODE          m_Camera;
	vector<CAMNODE>  m_vCamList;
};

class TerrPhotoFile_LIB CTerrImgList
{	
public:	
	void               CreateList(vector<RGSIMGLIST>& vImglist);	
	BOOL               LoadListFile(const char* lpstrPath);
	BOOL               SaveListFile(const char* lpstrPath);	

	CTerrImgList();
	virtual ~CTerrImgList();

public:
	vector<RGSIMGLIST>  m_vImglist;
};

class TerrPhotoFile_LIB CRgsFile
{	
public:	
	static int         FindStationSN(int nStation,vector<LASERORI>& vStationOri);
	static BOOL        LoadPairFile(const char* pstrFilePath,RGSPAIR* pStripPair);
	static BOOL        SavePairFile(const char* pstrFilePath,RGSPAIR* pStripPair);	
	static BOOL        RoughRgs4Obv(vector<RGSTIENODE>& vObvlst,RGSPAIR& TiePair);
	static BOOL        RoughRgs4Pair(vector<string>& vStrPair,const char* lpstrOri);
	static BOOL        CalTransform(vector<RGSTIENODE>& vObvlst,vector<KPTNODE>& vKptlst,double *pTransform,int *pFlag);
	static BOOL        SumLASTransform(const char* lpstrInLAS,const char* lpstrOutLAS,double *pTransform);

	BOOL               LoadPairList(vector<string>& vStrPair);
	BOOL               LoadObvFile(const char* pstrFilePath);
	BOOL               SaveObvFile(const char* pstrFilePath);
	BOOL               LoadOriFile(const char* pstrFilePath);
	BOOL               SaveOriFile(const char* pstrFilePath);

	BOOL               ResizeObvBuff(int nSize);
	BOOL               ResizePairBuff(int nSize);
	BOOL               ResizeOriBuff(int nSize);

	CRgsFile();
	virtual ~CRgsFile();	

public:
	vector<RGSTIENODE> m_vObvLst;
	vector<RGSPAIR>    m_vPairLst;
	vector<LASERORI>   m_vTrsLst;
};

class TerrPhotoFile_LIB CKptFile
{
public:
	BOOL             LoadKpt4File(const char* lpstrPath);
	BOOL             SaveKpt2File(const char* lpstrPath);
	void             SetKptLst(vector<KPTNODE>& vKptlst);

	CKptFile();
	virtual ~CKptFile();

public:
	vector<KPTNODE>  m_vKptlst;	
};

#endif // TerrPhotoFile_h__