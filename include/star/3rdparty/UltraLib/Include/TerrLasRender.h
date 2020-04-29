/********************************************************************
       Author:		           ZQW	
       File Name:	           TerrLasRender
	   File Type:	           h
       Created Date:	       2011/8/29
	   Created Time:	       21:45      
*********************************************************************/
#ifndef TerrLasRender_h_ZQW_2011_8_29_21_45_DEF
#define TerrLasRender_h_ZQW_2011_8_29_21_45_DEF

#include "TerrGlBase.h"
#include "TerrLasFile.h"
#include "TerrVectFile.h"

#ifndef TerrLasRender_LIB 
   #define TerrLasRender_LIB _declspec(dllimport)
   #ifdef _DEBUG
      #pragma comment(lib,"TerrLasRenderD.lib")
      #pragma message("Automatically linking with TerrLasRenderD.lib")
   #else
      #pragma comment(lib,"TerrLasRender.lib")
      #pragma message("Automatically linking with TerrLasRender.lib")
   #endif
#endif

#ifndef  _RENDERCLOUD
#define  _RENDERCLOUD

//QctTree index;
   typedef struct tagOCTNODE{
	   int            nStart,nNum;
	   int            Child[8],Father; 
	   short int      bLeaf,nLevel;
	   float          fMinX,fMinY,fMinZ; 
	   float          fMaxX,fMaxY,fMaxZ; 
   }OCTNODE,*POCTNODE;
   typedef struct tagOCTLAYER{
	   int            nPtNum,nDepth,nLeaf;
	   int            nOctLen,nBufLen;
	   float          fCubeSize,fGSD;	   
	   float          fMinX,fMinY,fMinZ; //for Box;
	   float          fMaxX,fMaxY,fMaxZ; //for Box;
	   double         xOffset,yOffset,zOffset;
	   PLASERNODE*    ppCldLst;
	   POCTNODE       pOctNode;
   }OCTLAYER,*POCTLAYER;
#endif

class TerrLasRender_LIB CShaderMap
{
public:
	enum MapMode{eStrip=0,eVERTICAL=1,eCLASSIFY=2,eRETURNS=3,eINTENSITY=4,eIMAGERGB=5,};
	enum ColorType{eHSV=0,eBLUERED=1,eEARTHSKIN=2,eDAYNIGHT=3,};	
	enum EchoType{eSIGLE=0,eFIRST=1,eMIDLE=2,eLAST=3,eAllReturns=4,};

	static int             GetEchoType(BYTE Return,BYTE ReturnNumber);

public:	
	void                   SetMapMode(int nMapMode=eVERTICAL){m_nMapMode=nMapMode;};
	void                   SetColorType(int nColorType=eBLUERED){m_nColorType=nColorType;};
	void                   SetZMaxMin(float fMaxZ,float fMinZ);	
	void                   SetIntensMaxMin(int nMaxIntens,int nMinIntens);
	void                   SetStripColor(int nStrip,BYTE Red,BYTE Green,BYTE Blue);
	void                   SetClassColor(int nClass,BYTE Red,BYTE Green,BYTE Blue);
	void                   SetEchoColor(int nReturn,BYTE Red,BYTE Green,BYTE Blue);
	void                   SetDefautColor(BYTE Red,BYTE Green,BYTE Blue);

	int                    GetMapMode(){return m_nMapMode;};
	int                    GetColorType(){return m_nColorType;};	
	float                  GetZMax(){return m_fMaxZ;}
	float                  GetZMin(){return m_fMinZ;}
	PVECT3F                GetStripColor(int nStrip);
	PVECT3F                GetVerColor(float fZValue);
	PVECT3F                GetEchoColor(int nReturnType);
	PVECT3F                GetClassColor(int nClassType);
	PVECT3F                GetIntenColor(int nIntensity);

	CShaderMap();
	virtual ~CShaderMap();
	
protected:
	VECT3F                 m_DefautColor;
	VECT3F                 m_HSVIdx[256],m_BlueRedIdx[256],m_EarthSkin[256],m_DayNight[256];
	VECT3F                 m_ReturnIdx[64],m_ClassIdx[64],m_StripIdx[256],m_IntenIdx[256];
	int                    m_nMapMode,m_nColorType;
	float                  m_fMinZ,m_fMaxZ;
	float                  m_fStep,m_fEarthStep;
	float                  m_fMinI,m_fMaxI,m_fIStep;

private:
	void                   InitStripColor();
	void                   InitVerColor();	
	void                   InitClassColor();
	void                   InitReturnColor();
	void                   InitIntensColor();
};
class TerrLasRender_LIB CLASTopRender : public C2DOper
{
public:
	BOOL                   AttachCld(CLasMgr *pLasMgr);
	void                   DetachCld();
	BOOL                   IsCldAttached(){return m_bCldAttached;};
	BOOL                   AttachGrid(CGridMgr *pGridMgr);
	void                   DetachGrid();
	BOOL                   IsGridAttached(){return m_bGridAttached;};	

	void                   Render();
	BOOL                   CreateCldLayer(int nMaxBlkNum=30000);
	BOOL                   IsCreated(){return m_bCreated;};
	BOOL                   IsLayerInited(){return (m_bGridAttached||m_bCldAttached);};

	BOOL                   PickVisiable(LASERNODE **ppPtLst,int *nPtNum);
	BOOL                   PickSection(float *pSectionX,float *pSectionY,LASERNODE **ppPtLst,int *nPtNum);
	BOOL                   PickRect(RECTBOX *pRect,LASERNODE **ppPtLst,int *nPtNum);
	BOOL                   PickPolygon(POLYGON *pPolygon,LASERNODE **ppPtLst,int *nPtNum);
	PLASERNODE             PickPoint(int ClntX,int ClntY);

	OCTLAYER*              GetCldLayer(){return (&m_OctLay);};
	int*                   GetSelectLst(){return m_pSelLst;};
	int                    GetSelectSize(){return m_nSelNum;};
	CShaderMap*            GetShaderMap(){return (&m_ShadeMap);};
	BOOL                   GetDrawGridFlag(){return m_bDrawGrid;};	
	BOOL                   GetDrawCldFlag(){return m_bDrawCld;};
	BOOL                   GetDrawGcpFlag(){return m_bDrawGcp;};
	float                  GetPointSize(){return m_fPointSize;}

	void                   SetPointSize(float fPointSize){m_fPointSize=fPointSize;};
	void                   SetMaxScreenNum(int nMaxScreenNum);
	void                   SetFitViewFlag(BOOL bFitView){m_bFitView=bFitView;};
	void                   SetDrawGridFlag(BOOL bDrawGrid){m_bDrawGrid=bDrawGrid;};
	void                   SetDrawCldFlag(BOOL bDrawCld){m_bDrawCld=bDrawCld;};
	void                   SetDrawGcpFlag(BOOL bDrawGcp){m_bDrawGcp=bDrawGcp;};
	void                   SetDrawPickFlag(BOOL bDrawPick){m_bDrawPick=bDrawPick;};
	void                   SetPickPoint(float x,float y,float z){m_fPickPt[0]=x;m_fPickPt[1]=y;m_fPickPt[2]=z;};

	void                   CreateList();
	void                   DestoryList();

	CLASTopRender();
	virtual ~CLASTopRender();

public:
	CRgsFile               m_tieFile;
	CLasMgr               *m_pLasMgr;	
	LASERNODE            **m_ppPickLst;
	int                    m_nPickNum;
	BOOL                   m_bOperating;
	float                  m_fPickPt[4];	

protected:
	BOOL                   m_bCreated;
	BOOL                   m_bCldAttached;
	BOOL                   m_bGridAttached;		
	BOOL                   m_bDrawCld;
	BOOL                   m_bDrawGrid;	
	BOOL                   m_bDrawGcp;
	BOOL                   m_bFitView;
	BOOL                   m_bDrawPick;
	BOOL                   m_bCreatLsted;

	OCTLAYER               m_OctLay;
	CShaderMap             m_ShadeMap;		
	CGridMgr              *m_pGridMgr;
	int                   *m_pSelLst,m_nSelNum;	
	float                  m_fPointSize;
	unsigned int           m_nGCPID;
	float                  m_fPtLen;

private:	
	void                   ClearLayer();
	void                   DrawRoutine(PRECTBOX pWndBox,OCTLAYER* pLayer,int nFather,int *pSelLst,int *nSelNum,int nSkip,BOOL bOperating);
	void                   DrawBlock(PLASERNODE *ppCldLst,int nStart,int nNum,int nSkip=1);
};

class TerrLasRender_LIB CSectionRender : public C2DOper
{
public:	
	BOOL                   CreateRender(LASERNODE **ppCldLst,int nCldSize,CLasMgr* pLasMgr,int nMaxBlkNum=512);
	void                   DeleteRender();	
	BOOL                   IsCreated(){return m_bCreated;};

	void                   Render();
	BOOL                   ClassifyRect(RECTBOX *pRect,int nDesType,LASERNODE **ppPtLst,int *nPtNum);
	BOOL                   ClassifyPoly(POLYGON *pPolygon,int nDesType,LASERNODE **ppPtLst,int *nPtNum);

	void                   ZoomSection(int nClntX,int nClntY,float fRatio);
	void                   ZoomFit(float xMin,float xMax,float yMin,float yMax,float zMin,float zMax);
	int                    GetBuffSize(){return m_nBufSize;};
	POCTLAYER              GetLayerIdx(){return (&m_OctLay);};	
	int*                   GetSelectLst(int *nLstSize){*nLstSize=m_nSelNum;return m_pSelLst;}
	CShaderMap*            GetShaderMap(){return (&m_ShadeMap);}
	float                  GetPointSize(){return m_fPointSize;}
	void                   SetPointSize(float fPointSize){m_fPointSize=fPointSize;}
	void                   ReSetView(BOOL bReset=TRUE){m_bResetView=bReset;}
	void                   SetProfileVector(PVECT3F pVStart,PVECT3F pVEnd){m_vSectionSt=pVStart[0];m_vSectionEnd=pVEnd[0];}

	CSectionRender();
	virtual ~CSectionRender();

public:
	CLasMgr               *m_pLasMgr;
	PLASERNODE            *m_ppBufLst;
	int                    m_nBufSize,m_nCldSize;

protected:
	BOOL                   m_bCreated;
	BOOL                   m_bResetView;
	OCTLAYER               m_OctLay;
	CShaderMap             m_ShadeMap;	

	int                   *m_pSelLst,m_nSelNum;	
	float                  m_fPointSize;
	float                  m_fGridSize;
	VECT3F                 m_vSectionSt,m_vSectionEnd;

private:	
	BOOL                   ResizeCldBuffer(int nCldSize);
	void                   ClearLayer();
	void                   DrawRoutine(OCTLAYER* pLayer,int nFather,int *pSelLst,int *nSelNum);
	void                   DrawBlock(PLASERNODE *ppCldLst,int nStart,int nNum,int nSkip=1);
};
class TerrLasRender_LIB COctRender : public C3DOper
{
public:
	BOOL                   AttachCld(PLASERHDR pCldHdr,LASERNODE **ppCldLst);
	void                   DetachCld();
	BOOL                   IsAttached(){return m_bAttached;};

	BOOL                   CreateRender(int nMaxBlkNum=30000,BOOL bOrthView=FALSE);	
	BOOL                   IsCreated(){return m_bCreated;};

	void                   Render();
	void                   DrawLines(vector<LASERNODE>& vPoint);
	PLASERNODE             PickPoint(int ClntX,int ClntY);
	BOOL                   ClassifyPoly(POLYGON *pPolygon,int *OrgType,int nDesType);
	BOOL                   FitingSphere(int nFromClass,float fSphereRdu,double *a,double *b,double *c,double *uWeight,BOOL bGrossRemove=TRUE);
	BOOL                   FitingSphere(PLASERNODE pSeedNode,float fSearchRang,float fSphereRdu,double *a,double *b,double *c,double *uWeight,int nSpherePtClass=eModelKey,BOOL bGrossRemove=TRUE);

	OCTLAYER*              GetLayer(){return (&m_OctLay);};
	int*                   GetSelLst(){return m_pSelLst;};
	int                    GetSelNum(){return m_nSelNum;};
	CShaderMap*            GetShaderMap(){return (&m_ShadeMap);}
	int*                   GetClassFlag(){return m_bClassFlag;};
	float                  GetPointSize(){return m_fPointSize;}
	void                   SetPointSize(float fPointSize){m_fPointSize=fPointSize;}
	void                   SetClassFlag (int nClass,int bShowFlag=1);
	void                   SetEchoFlag(int nEchoType,int bShowFlag=1);	
	void                   ReSetView(BOOL bReset=TRUE){m_bResetView=bReset;};
	void                   SetViewMode(int nFitMode=C3DOper::eTopView);

	void                   CreateList();
	void                   DestoryList();

	COctRender();
	virtual ~COctRender();

public:
	LASERHDR              *m_pCldHdr;
	LASERNODE            **m_ppCldLst;
	LASERNODE            **m_ppPickLst;
	int                    m_nPickNum;
	float                  m_fPickPt[4];
	BOOL                   m_bOperating;
	BOOL                   m_bOrthView;
	BOOL                   m_bDrawCld;
	BOOL                   m_bDrawSphere;
	BOOL                   m_bCreatLsted;

protected:
	BOOL                   m_bAttached,m_bCreated;
	BOOL                   m_bResetView;
	int                    m_nFitMode;
	CShaderMap             m_ShadeMap;
	OCTLAYER               m_OctLay;

	int                   *m_pSelLst,m_nSelNum;
	int                    m_bClassFlag[64],m_bEchoFlag[64];	
	float                  m_fPointSize;

	unsigned int           m_nSphereID;
	double                 m_Sphere[4];

private:	
	void                   ClearLayer();
	void                   DrawRoutine(OCTLAYER* pLayer,int nFather,int *pSelLst,int *nSelNum,double *dView,double dScrRatio,BOOL bOperating);
	void                   DrawBlock(PLASERNODE *ppCldLst,int nStart,int nNum,int nSkip=1);	
};
class TerrLasRender_LIB CTriRender : public C3DOper
{
public:
	enum eTriMode{eVertex=0,eMesh=1,eSolid=2,};

	BOOL                   Attach(void *pTriMesh,CLasMgr *pLasMgr);
	void                   Detach();
	BOOL                   IsAttached();

	void                   Render();
	void                   ReCalcuateNrml();
	BOOL                   UpdateMesh(int *Class);
	BOOL                   UpdateMesh(LASERNODE **ppLasList,int nListSize);
	void                   SetRenderMode(int nMode=eMesh);
	void                   ReSetView(BOOL bReset=TRUE);
	PLASERNODE             PickPoint(int ClntX,int ClntY);

	float                  GetMinX();
	float                  GetMaxX();
	float                  GetMinY();
	float                  GetMaxY();
	float                  GetMinZ();
	float                  GetMaxZ();	

	CTriRender();
	virtual ~CTriRender();

public:	
	CLasMgr               *m_pLasMgr;
	CShaderMap             m_ShadeMap;	
	void                  *m_pTriMesh;
	int                    m_nRenderMode;
	BOOL                   m_bAttached;
	BOOL                   m_bResetView;
	BOOL                   m_bCreated;	
	float                  m_fMdlCenter[4];
	float                  m_fPointSize;

private:
	void                   DrawFaceList();
};
class TerrLasRender_LIB    CVectorRender
{
public:
	enum eElementType
	{	
		ePointType    = 0,
		eLineType     = 1,
		eAreaType     = 2,			
	};	
	typedef struct tagCoordList{
		vector<PT3F>    vCoordlist;
		int             nElemType;
	}COORDLIST,*PCOORDLIST;

	BOOL                   AddElement(vector<PT3F>& vCoordlist,int nElemType);
	BOOL                   RemoveElement(RECTBOX *pRtBox);
	void                   Clear();

	BOOL                   Import4File(const char* lpstrPath);
	BOOL                   Export2File(const char* lpstrPath,int nElemtype,double xOff=0.0,double yOff=0.0,double zOff=0.0);

	void                   Render();

	//for editor;

	CVectorRender();
	virtual ~CVectorRender();

public:	
	double                 m_Offset[3];
	vector<PCOORDLIST>     m_vElement;
	float                  m_fPointSize;
	VECT3F                 m_vectColor[3];
	int                    m_bDrawFlag[3];

private:
	void                   DrawVector();
};

#endif // TerrLasRender_h__
