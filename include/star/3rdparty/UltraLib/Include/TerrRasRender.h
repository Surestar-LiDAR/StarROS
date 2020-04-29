/********************************************************************
       Author:		           ZQW	
       File Name:	           TerrRasRender
	   File Type:	           h
       Created Date:	       2012/12/12
	   Created Time:	       16:19      
*********************************************************************/
#ifndef TerrRasRender_h_ZQW_2012_12_12_16_19_DEF
#define TerrRasRender_h_ZQW_2012_12_12_16_19_DEF

#include "TerrGlBase.h"
#include <vector>
using namespace std;

#ifndef TerrRasRender_LIB 
   #define TerrRasRender_LIB _declspec(dllimport)
   #ifdef _DEBUG
      #pragma comment(lib,"TerrRasRenderD.lib")
      #pragma message("Automatically linking with TerrRasRenderD.lib")
   #else
      #pragma comment(lib,"TerrRasRender.lib")
      #pragma message("Automatically linking with TerrRasRender.lib")
   #endif
#endif

class TerrRasRender_LIB CImgRender : public C2DOper
{
public:
	CImgRender();
	virtual ~CImgRender();		

	BOOL                   Attach(void *pRaster,int nRed=1,int nGreen=2,int nBlue=3,int nBlkSize=256);
	void                   Detach();
	BOOL                   IsAttached(){return m_bAttached;};
	
	void                   Render(BOOL bUpdate=FALSE);
	void                   ClearTexture();
	float                  GetMapWidth(){return (float(m_nCols));}
	float                  GetMapHeight(){return (float(m_nRows));}

public:
	virtual void           ZoomFit(float xMin,float xMax,float yMin,float yMax,float zMin=0.0f,float zMax=0.0f);
	virtual void           ZoomReal(float xMin,float xMax,float yMin,float yMax,float zMin=0.0f,float zMax=0.0f);

protected:
	BOOL                   m_bAttached,m_bReAttached;		
	int                    m_nCols,m_nRows,m_nBandNum;
	int                    m_BandList[3];
	int                    m_nBlkSize;

private:	
	void                  *m_pRaster;	
	BYTE                  *m_pBuffer;
	RECTBOX               *m_pRectlst;
	int                    m_nLstSize;
	int                    m_nMaxSize;		
};

#endif // TerrRasRender_h__