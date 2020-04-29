/********************************************************************
       Author:		           ZQW	
       File Name:	           TerrVectFile
	   File Type:	           h
       Created Date:	       2013/5/30
	   Created Time:	       8:44      
*********************************************************************/
#ifndef TerrVectFile_h_ZZQUAN_2013_5_30_8_44_DEF
#define TerrVectFile_h_ZZQUAN_2013_5_30_8_44_DEF

#pragma   warning(disable:4786)
#pragma   warning(disable:4251)
#pragma   warning(disable:4089)
#pragma   warning(disable:4996)
#pragma   warning(disable:4099)

#include <vector>
#include <map>
using namespace std;

//shp文件说明：
//至少包含3个文件：shp,shx,dbf,其中dbf存储属性；
//单文件仅能存储一类型数据(点-线-面),但可储存多个对象,如多个点、多条线、多个面等;
//属性值都作为字符串存取,必须指定数值类型。

#ifndef   _SHPGEOMETRY
#define   _SHPGEOMETRY
typedef enum enumGeometryType
{
	enum_Geo_Null=0,
	enum_Geo_Pt=1,
	enum_Geo_Line=2,
	enum_Geo_Area=3,
	enum_Geo_Text=4,
}eGeometryType;
typedef enum enumOGRFieldType
{
	eOFTInteger = 0,       /** Simple 32bit integer*/
	eOFTIntegerList = 1,   /** List of 32bit integers*/ 
	eOFTReal = 2,          /** Double Precision floating point*/
	eOFTRealList = 3,      /** List of doubles*/
	eOFTString = 4,        /** String of ASCII chars*/
	eOFTStringList = 5,    /** Array of strings */
	eOFTWideString = 6,    /** deprecated*/ 
	eOFTWideStringList = 7,/** deprecated*/ 
	eOFTBinary = 8,        /** Raw Binary data*/ 
	eOFTDate = 9,          /** Date*/
	eOFTTime = 10,         /** Time*/ 
	eOFTDateTime = 11,     /** Date and Time*/ 
	eOFTMaxType = 11
}eOGRFieldType;
typedef struct tagPointXYZ
{
	double x;
	double y;
	double z;
	tagPointXYZ(){x=0.0f,y=0.0f,z=0.0f;}
	tagPointXYZ(double dx,double dy,double dz=0.0f){x=dx,y=dy,z=dz;}
}PointXYZ,*PPointXYZ;
typedef struct tagGeometryData
{
	eGeometryType					    m_eType;		//数据类型，点线面;
	std::string							m_strLayerName;	//数据转换层名;	
	std::string							m_strText;		//数据文字类型，文字内容;
	std::vector<PointXYZ>				m_vCoords;		//坐标点串;
	std::map<std::string,std::string>	m_mapFieldAtt;	//转换数据属性，如点ID、名称等;
	tagGeometryData()
	{
		m_eType = enum_Geo_Null;
		m_strLayerName = "";
		m_strText = "";
	}
}GeometryData,*PGeometryData;
#endif

//for mesh file read/write;
#ifndef   _MDLGEOMETRY
#define   _MDLGEOMETRY
#define   MDLNAMELEN  128
typedef struct tagMDLVETX
{ 
	float      x,y,z;
}MDLVETX,*PMDLVETX;
typedef struct tagMDLNORMAL
{ 
	float      nx,ny,nz;
}MDLNORMAL,*PMDLNORMAL;
typedef struct tagMDLTEXUCRD
{
	float      u,v;
}MDLTEXUCRD,*PMDLTEXUCRD;
typedef struct tagMDLTRIANGLE
{	
	int nVetxIdx[3];
	int nNeibTri[3];
}MDLTRIANGLE,*PMDLTRIANGLE;
typedef struct tagMDLBOXD
{
	double  xMin,xMax;
	double  yMin,yMax;
	double  zMin,zMax;
}MDLBOXD,*PMDLBOXD;
typedef struct tagMDLBOXF
{
	float  xMin,xMax;
	float  yMin,yMax;
	float  zMin,zMax;
}MDLBOXF,*PMDLBOXF;
#endif

#ifndef TerrVectFile_LIB 
   #define TerrVectFile_LIB _declspec(dllimport)
   #ifdef _DEBUG
      #pragma comment(lib,"TerrVectFileD.lib")
      #pragma message("Automatically linking with TerrVectFileD.lib")
   #else
      #pragma comment(lib,"TerrVectFile.lib")
      #pragma message("Automatically linking with TerrVectFile.lib")
   #endif
#endif

class TerrVectFile_LIB CShpFile
{
public:	
	static void           Init6Transform(double *pTransform);
	static void           Init7Transform(double *pTrasnform);
	static void           StatisticsBox(vector<PGeometryData>& vData,MDLBOXD* pBox);

	//文件I/O操作；
	BOOL                  LoadShpFile(const char* lpstrFilePath);
	BOOL                  SaveShpFile(const char* lpstrFilePath,eGeometryType eType);
	void                  Clear();
	
	//创建几何体及字段信息；
	PGeometryData         CreateGeometryElement(eGeometryType eType);
	void                  SetGeometryCoord(PGeometryData pData,PointXYZ point);
	void                  SetGeometryFiledValue(PGeometryData pData,const char* lpstrKey,const char* lpstrValue);
	void                  SetGeometryLayerName(PGeometryData pData,const char* lpstrLayerName);
	void                  SetGeometryText(PGeometryData pData,const char* lpstrText);
	//设置字段类型；
	void                  AddFieldType(const char* lpstrKey,eOGRFieldType eFieldType);		

	CShpFile();
	virtual ~CShpFile();

public:
	vector<PGeometryData>     m_vData;
	map<string,eOGRFieldType> m_mapFieldsType;
	char                      m_strWkt[512];
	char                      m_strFilePath[512];
	double                    m_transform[7];
	MDLBOXD                   m_MdlBox;			
};

class TerrVectFile_LIB CMeshFile
{
public:	
	static BOOL           SaveTri2Fbx(void *pTriMesh,const char* lpstrFbxPath);
	static BOOL           SaveTri2Obj(void *pTriMesh,const char* lpstrObjPath);
	static BOOL           SaveTri2Mesh(void *pTriMesh,double *pTransform,const char* lpstrMeshPath);	
	static void           CalModelBox(vector<MDLVETX>& vVetxlist,MDLBOXF *pMdlBox);

	BOOL                  LoadMeshFile(const char* lpstrPath);
	BOOL                  SaveMeshFile(const char* lpstrPath);
	void                  Clear();

	CMeshFile();
	virtual ~CMeshFile();

public:
	//for global;
	double                m_transform[7];
	double                m_Offset[3];
	MDLBOXF               m_MdlBox;	
	vector<MDLVETX>       m_vVetxlst;
	vector<MDLNORMAL>     m_vNrmllst;	
	vector<MDLTRIANGLE>   m_vTrilst;
	vector<MDLTEXUCRD>    m_vTextlst;
};

#endif // TerrVectFile_h__