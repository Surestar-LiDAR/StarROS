// TerrGeoCvt.h : main header file for the TerrGeoCvt DLL
/********************************************************************
       Author:		           ZQW	
       File Name:	           TerrGeoCvt
	   File Type:	           h
       Created Date:	       2010/4/16
	   Created Time:	       11:45      
*********************************************************************/
#ifndef TerrGeoCvt_h_ZQW_2010_4_16_11_45_DEF
#define TerrGeoCvt_h_ZQW_2010_4_16_11_45_DEF

#ifndef TerrGeoCvt_LIB    
     #define TerrGeoCvt_LIB  __declspec(dllimport)
     #ifdef _DEBUG
     #pragma comment(lib,"TerrGeoCvtD.lib") 
     #pragma message("Automatically linking with TerrGeoCvtD.lib") 
     #else
     #pragma comment(lib,"TerrGeoCvt.lib") 
     #pragma message("Automatically linking with TerrGeoCvt.lib") 
     #endif
#endif

#ifndef    _GEOCVTPAR 
#define    _GEOCVTPAR
enum ProjectionType
{
	GAUSS_PROJECTION    = 0,   //高斯－克吕格投影
	UTM_PROJECTION      = 1,   //通用横轴墨卡托投影
	TM_PROJECTION       = 2,   //横轴墨卡托投影	
	MERCATOR_PROJECTION = 3,   //墨卡托投影	
};
enum GRAVDZ_TYPE
{
    GT_GEODETIC    = 0,        //经纬度格网改正类型
	GT_PROJECTION  = 1,        //平面格网改正类型
};
typedef struct tagELLIPSOID
{
	char    strName[64];       //椭球名称
	double  eps_A;             //椭球长半轴
	double  eps_B;	           //椭球短半轴
	double  eps_flat;          //椭球扁率
	double  e1_square;         //第一偏心率的平方
	double  e2_square;         //第二偏心率的平方
}ELLIPSOID,*PELLIPSOID;
typedef struct tagGRAVSPHEROID //球体重力异常类型
{   
	int    nType;
	double StartLat,StartLon;
	double dLon,dLan;         
	double StartX,startY;
	double dx,dy;
	float *pDVer;
}GRAVSPHEROID,*PGRAVSPHEROID;
typedef struct tagCOORDPARA   //坐标系统框架
{          
	ELLIPSOID     ellipsoid;
	int           nProjection;
	int           nVerData;
	double        dCenMeridian;//Degree  
	double        dCenLatitude;//Degree
	double        dEastOffset;
	double        dNorthOffset;
	double        dProjectRatio;
	char          strRev[512];
}COORDPARA,*PCOORDPARA;
#endif

/*/////////////////////////////////////////////////////////////////////////
//采用布尔莎模型实现椭球间的转换；
//参数列表(Tx,Ty,Tz,Ex,Ey,Ez,dK)
//Scale=1.0+dK；
//pR[0] = 1.0;    pR[1] = dAngZ; pR[2] = -dAngY;
//pR[3] = -dAngZ; pR[4] = 1.0;   pR[5] = dAngX;
//pR[6] = dAngY;  pR[7] = -dAngX;pR[8] = 1.0;
//Des=T+Scale*pR*Org
/////////////////////////////////////////////////////////////////////////*/

class TerrGeoCvt_LIB CGeoConvert
{
public:
	static void          CalEllipsoid(double Eps_A,double Eps_B,PELLIPSOID pEllipsoid); 
    static int           CXCYCZ_to_LBH(PELLIPSOID pEllipsoid,double CX,double CY,double CZ,double* L,double* B,double* H);
    static int           LBH_to_CXCYCZ(PELLIPSOID pEllipsoid,double L,double B,double H,double* CX,double* CY,double* CZ);
    static void          CXCYCZ1_to_CXCYCZ2(double *Par7,double CX1,double CY1,double CZ1,double *CX2,double* CY2,double *CZ2);
	static void          ENH_to_LBH(PCOORDPARA pCoordPar,double easting,double northing,double height,double* L,double* B,double* H);
    static void          LBH_to_ENH(PCOORDPARA pCoordPar,double L,double B,double H,double* easting,double* northing,double* height);

	static BOOL          LoadPrj4File(const char* lpstrCvtPath,PCOORDPARA pCoordPar);	
	static BOOL          Load7Par4File(const char* lpstrCvtPath,double *p7Par);	
	static BOOL          SavePrj2File(const char* lpstrCvtPath,PCOORDPARA pCoordPar);	
	static BOOL          Save7Par2File(const char* lpstrCvtPath,double *p7Par);	 
	static void          SaveDeclaration(const char* lpstrCvtPath);
	
public:
    CGeoConvert();
    virtual ~CGeoConvert();

	BOOL                 Init(PCOORDPARA pOrgPar,PCOORDPARA pTgtPar,double *p7Trans,PGRAVSPHEROID pGravdZ=NULL);
	BOOL                 IsInit(){return m_bInit;}   

	BOOL                 CenOrg2ProjTgt(double CX,double CY,double CZ,double *easting,double *northing,double *height);
	BOOL                 ProjOrg2CenTgt(double easting,double northing,double height,double *CX,double *CY,double *CZ);
	BOOL                 LBHOrg2ProjTgt(double L,double B,double H,double *easting,double *northing,double *height);
	BOOL                 ProjOrg2LBHTgt(double easting,double northing,double height, double *L,double *B,double *H);
	BOOL                 CenOrg2ProjTgtMatrix(double *RtMatrix,double *Phi,double *Omega,double *Kapa);

	ELLIPSOID&           GetWGS84Ellipsoid(){return m_WGS84;};
	ELLIPSOID&           GetCGCS2000Ellipsoid(){return m_CGCS2000;};
	ELLIPSOID&           GetKrasyEllipsoid(){return m_Krassovsky;};
	ELLIPSOID&           GetIUGG1975Ellipsoid(){return m_IUGG1975;};
		
protected:
	BOOL                 m_bInit;
	COORDPARA            m_OrgPar,m_TgtPar;
	PGRAVSPHEROID        m_pGravdZ;
	double               m_7Trans[7];

	ELLIPSOID            m_WGS84;	
	ELLIPSOID            m_CGCS2000;
	ELLIPSOID            m_Krassovsky;
	ELLIPSOID            m_IUGG1975;
};

#endif // TerrGeoCvt_h__
