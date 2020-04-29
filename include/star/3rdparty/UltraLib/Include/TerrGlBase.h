// TerrGlBase.h : main header file for the TerrGlBase DLL
/********************************************************************
       Author:		           ZQW	
       File Name:	           TerrGlBase
	   File Type:	           h
       Created Date:	       2010/5/21
	   Created Time:	       20:08      
*********************************************************************/
#ifndef TerrGlBase_h_ZQW_2010_5_21_20_08_DEF
#define TerrGlBase_h_ZQW_2010_5_21_20_08_DEF

#include "BasicDef.h"
#include "gl/glVir.h"

#ifndef   FONTLIST
#define   FONTLIST    96
#endif

#ifndef TerrGlBase_LIB    
     #define TerrGlBase_LIB  __declspec(dllimport)
     #ifdef _DEBUG
     #pragma comment(lib,"TerrGlBaseD.lib") 
     #pragma message("Automatically linking with TerrGlBaseD.lib") 
     #else
     #pragma comment(lib,"TerrGlBase.lib") 
     #pragma message("Automatically linking with TerrGlBase.lib") 
     #endif 
#endif

namespace nsVectorOp
{
	void  TerrGlBase_LIB vAdd  ( PVECT3F pV1,PVECT3F pV2,PVECT3F pRult );	
	void  TerrGlBase_LIB vMinus( PVECT3F pV1,PVECT3F pV2,PVECT3F pRult );
	void  TerrGlBase_LIB vMult ( PVECT3F pV1,float scale,PVECT3F pRult );
	void  TerrGlBase_LIB vDivid( PVECT3F pV1,float scale,PVECT3F pRult );
	void  TerrGlBase_LIB vCrossMult( PVECT3F pV1,PVECT3F pV2,PVECT3F pRult );
	float TerrGlBase_LIB vDotMult( PVECT3F pV1,PVECT3F pV2 );

	float TerrGlBase_LIB Distance ( PVECT3F pV1,PVECT3F pV2 );
	float TerrGlBase_LIB Distance ( float x1,float y1,float z1,float x2,float y2,float z2 );
	float TerrGlBase_LIB Distance2( float x1,float y1,float z1,float x2,float y2,float z2 );
	float TerrGlBase_LIB vModulus ( PVECT3F pV );
	void  TerrGlBase_LIB vNormalize( PVECT3F pV );
	void  TerrGlBase_LIB vNormal( PVECT3F pVlst,PVECT3F pRult,int lstSize=3 );
	float TerrGlBase_LIB vVectAngl( PVECT3F pV1,PVECT3F pV2 );

	void  TerrGlBase_LIB RotateVect( float angle,PVECT3F pvStart,PVECT3F pvEnd,PVECT3F pvAxis );
};

class TerrGlBase_LIB CCameraUtl
{
public:
	enum FrustumSide
	{
		frRIGHT	  = 0,		// The RIGHT side of the frustum
		frLEFT	  = 1,		// The LEFT	 side of the frustum
		frBOTTOM  = 2,		// The BOTTOM side of the frustum
		frTOP     = 3,		// The TOP side of the frustum
		frNEAR	  = 4,		// The FRONT side of the frustum
		frFAR	  = 5,		// The BACK	side of the frustum		
	}; 

	void                   LevelFBCamera(float speed);	
	void                   LevelLRCamera(float speed);
	void                   ViewFBCamera(float speed);

	void                   CalcFrustum(double *pProjMatx,double *pModViewMatx);
	void                   CalcFrustum(double xMin,double xMax,double yMin,double yMax);
	BOOL                   PtInFrustum(float x, float y, float z);
	BOOL                   SphereInFrustum(float x, float y, float z, float radius);
	BOOL                   CubeInFrustum(float x, float y, float z, float size);

	void                   ApplyCamera();

	CCameraUtl();
	virtual ~CCameraUtl();	
	
public:
	double                 m_Frustum[6][4];	
	VECT3F                 m_vPosition,m_vView,m_vUp;
};
class TerrGlBase_LIB CGlFontLst
{
public:
	BOOL                 Init(int nFtHei=16,LPCSTR lpstrFtName="Times New Roman");
	BOOL                 IsInit(){return m_bInit;}
	void                 SetTextColor(float fR,float fG,float fB,float fAlpa);
	void                 DestoryFont();

	void                 PrintStr2D(const char *strText,int ClnX,int ClnY,int ClnW,int ClnH);
	void                 PrintStr3D(const char *strText,float x,float y,float z);
	
	CGlFontLst();
	~CGlFontLst();

protected:
	BOOL                 m_bInit;
	DWORD                m_nListBase;
	HFONT                m_hFont;
	float                m_fR,m_fG,m_fB,m_fAlpa;
};

class TerrGlBase_LIB CGlFontBmp															
{
public:
	BOOL                IsInit(){return m_bInit;};
	BOOL                Init(int nSize=16,int nStrLen=512,char *fontName="Times New Roman");
	void                SetTextColor(float fR,float fG,float fB,float fAlpa);
	void                PrintStr2D(const char *string,int ClnX,int ClnY,int ClnW,int ClnH);	
	void                PrintStr3D(char *string,float x,float y,float z);

	CGlFontBmp();
	~CGlFontBmp();

protected:	
	BOOL                m_bInit;
	HFONT               m_hFont;
	PBITMAPINFO         m_pBmpInfo;
	UCHAR*              m_pBmpBits;
	int                 m_nStrLen;
	float               m_fR,m_fG,m_fB,m_fAlpa;	
};
class TerrGlBase_LIB CModelOper
{
public:	
	enum eMirrorType{eXMir=0,eYMir=1,eZMir=2,};

	void                   IdentityOperate();
	void                   OperateModel();	
	
	void                   SetContinueState(BOOL bContinue=FALSE);
	float*                 GetRotMx(){return m_fRotMx;}

	void                   MoveModel(float fTransX,float fTransY,float fTransZ,float fNrmlRatio=1.0f);
	void                   MoveModel(int nLastX,int nLastY,int nCurtX,int nCurtY,float fNrmlRatio=1.0f);	
	void                   ZoomModel(float fCenX,float fCenY,float fCenZ,float xStep,float yStep,float zStep);
	void                   RotModel(float fCenX,float fCenY,float fCenZ,int nLastX,int nLastY,int nCurtX,int nCurtY);
	void                   RotModel(float fCenX,float fCenY,float fCenZ,float fAxisX,float fAxisY,float fAxisZ,float fAngle);	
	void                   RotModel(float fCenX,float fCenY,float fAngle);
	void                   MirrorModel(float fCenX,float fCenY,float fCenZ,eMirrorType eMir=eYMir);

	CModelOper();
	virtual ~CModelOper();

public:
	CCameraUtl             m_Camera;
	float                  m_fModleMx[16];

protected:	
	float                  m_fCurtMx[16],m_fRotMx[16];
	int                    m_bContinue;
};
class TerrGlBase_LIB CSceneMgr
{
public:	
	enum eProjectionMode{ePerspective=0,eOrtho3D=1,};
	enum eRastMode{eImageTop=0,eLaserTop=1,eSection=2,};

	static void            DrawRastAxis(CGlFontLst* pFont,eRastMode eRm=eLaserTop,float fAxisLen=30.0f);	
	static void            Draw3DAxis(CGlFontBmp* pFont,float *pRotMx,float fAxisLen=30.0f);;
	static void            Draw3DAxis(CGlFontLst* pFont,float *pRotMx,float fAxisLen=30.0f);
	static void            Clnt2World(PVECT3F pVectLst,int nVectNum);	
	static void            World2Clnt(PVECT3F pVectLst,int nVectNum);

	static void            Clnt2World(double *x,double *y,double *z);
	static void            Clnt2World(float *x,float *y,float *z);
	static void            World2Clnt(double *x,double *y,double *z);
	static void            World2Clnt(float *x,float *y,float *z);

	static void            Clnt2View(double *x,double *y,double *z);
	static void            Clnt2View(float *x,float *y,float *z);
	static void            View2Clnt(double *x,double *y,double *z);
	static void            View2Clnt(float *x,float *y,float *z);

	static void            World2View(double *x,double *y,double *z);
	static void            World2View(float *x,float *y,float *z);
	static void            View2World(double *x,double *y,double *z);
	static void            View2World(float *x,float *y,float *z);
	
	static double          GetVerZoomRatio();
	static double          GetHorZoomRatio();

	void                   CreateScene(HDC hDC);
	void                   DestoryScene();
	BOOL                   IsSceneCreated(){return m_bCreated;};
	void                   DefaultRender(BOOL bDepthTest=TRUE,BOOL bShaderSmooth=TRUE);

	void                   SetViewPort(double ViewSx,double ViewSy,double ViewW,double ViewH);
	void                   SetOrthSpace(double Left,double Right,double Bottom,double Top,double zNear,double zFar);
	void                   SetFrustumAngle(double AngleVer,double zNear,double zFar);
	void                   SetFrustumDist(double Left,double Right,double Bottom,double Top,double zNear,double zFar);
	PVIEWPORT              GetViewPort(){return (&m_ViewPort);};
	PORTHPARA              GetOrthSpace(){return (&m_OrthPar);};
	PFRUSTUMANGLE          GetFrustumAngle(){return (&m_FrustumAngle);};
	PFRUSTUMDIST           GetFrustumDist(){return (&m_FrustumDist);};	

	void                   ConnectRC(int nBuffMode=GL_FRONT,BOOL bClearBuff=FALSE );
	void                   DisConnectRC(BOOL bSwapBuff=FALSE);
	HGLRC                  GetGLRC(){return (m_hRC);};
	HDC                    GetGLDC(){return (m_hDC);};

	CSceneMgr();
	virtual ~CSceneMgr();

public:	
	CModelOper             m_ModelOper;
	CGlFontLst             m_glFont;
	CGlFontBmp             m_bmpFont;
	int                    m_stProjMode;

protected:
	BOOL                   m_bCreated;
	HGLRC                  m_hRC;	
	HDC                    m_hDC;
	HPALETTE               m_hPalette;	

	VIEWPORT               m_ViewPort;
	ORTHPARA               m_OrthPar;
	FRUSTUMANGLE           m_FrustumAngle;
	FRUSTUMDIST            m_FrustumDist;
	double                 m_ProjMx[16],m_MdlView[16];
	int                    m_nGlViewPort[4];

private:
	BOOL                   SetupPixelFormat();
	void                   SetLogicalPalette();
};
class TerrGlBase_LIB C2DOper
{
public:
	enum eSampleType{eLINEAR=0,eNEAREST=1,};	

	virtual void           Render();

	virtual void           MoveRaster(int nLastX,int nLastY,int nCurtX,int nCurtY);
	virtual void           MoveRaster(float x,float y);
	virtual void           RotRaster (int nClntX,int nClntY,float fDegree);	
	virtual void           ZoomCustom(int nClntX,int nClntY,float fRatio);
	virtual void           ZoomIn(float fZoomStep);
	virtual void           ZoomOut(float fZoomStep);
	virtual void           ZoomRect(PRECT pZmRt );
	virtual void           ZoomFit(float xMin,float xMax,float yMin,float yMax,float zMin=0.0f,float zMax=0.0f);
	virtual void           ZoomReal(float xMin,float xMax,float yMin,float yMax,float zMin=0.0f,float zMax=0.0f);
	virtual void           MirrorRaster(int nClntX,int nClntY,int eMirMode=CModelOper::eYMir);

	virtual void           Client2Raster(float *fx,float *fy);
	virtual void           Raster2Client(float *fx,float *fy);
	virtual void           Client2View(float *fx,float *fy);
	virtual void           View2Client(float *fx,float *fy);
	virtual float          GetRasterZoomRatio();

	C2DOper();
	virtual ~C2DOper();

public:
	CSceneMgr              m_Scene;
};

class TerrGlBase_LIB C3DOper
{
public:
	enum FitMode{eTopView=0,eLeftView=1,eRightView=2,eFrontView=3,eBackView=4,eBottomView=5,};

	virtual void           Render();
		
	virtual void           MoveVector(int nLastX,int nLastY,int nCurtX,int nCurtY,float fNrmlRatio=1.0f);
	virtual void           RotateModel(int nLastX,int nLastY,int nCurtX,int nCurtY,float *Center4f);
	virtual void           ZoomModel(float fRatio,float *Center4f);	
	virtual void           ZoomFit(float xMin,float xMax,float yMin,float yMax,float zMin,float zMax);	

	void                   MakeLight_And_Material(BOOL bTwoSide=FALSE,BOOL bLocalView=FALSE);
	float*                 GetWholeAmbient() {return m_whole_Ambient;};
	float*                 GetLightPos()     {return m_light_position;};
	float*                 GetLightAmbient() {return m_light_Ambient;};
	float*                 GetLightDiffuse() {return m_light_Diffuse;};
	float*                 GetLightSpecular(){return m_light_Specular;};
	float*                 GetMaterialAmbient() {return m_mat_Ambient;};
	float*                 GetMaterialDiffuse() {return m_mat_Diffuse;};
	float*                 GetMaterialSpecular(){return m_mat_Specular;};
	float*                 GetMaterialColor(){return m_mat_Color;}

	C3DOper();
	virtual ~C3DOper();	

public:
	CSceneMgr              m_Scene;
	float                  m_fOperRatio;
	float                  m_whole_Ambient[4];
	float                  m_light_position[4];
	float                  m_light_Ambient[4];
	float                  m_light_Diffuse[4];
	float                  m_light_Specular[4];
	float                  m_mat_Ambient[4];
	float                  m_mat_Diffuse[4];
	float                  m_mat_Specular[4];
	float                  m_mat_Color[4];
};

#endif // TerrGlBase_h__
