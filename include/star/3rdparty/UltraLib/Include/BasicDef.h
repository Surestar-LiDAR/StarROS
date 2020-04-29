/********************************************************************
       Author:		           ZQW	
       File Name:	           BasicDef
	   File Type:	           h
       Created Date:	       2009/8/28
	   Created Time:	       10:50      
*********************************************************************/
#ifndef BasicDef_h_ZQW_2009_8_28_10_50_DEF
#define BasicDef_h_ZQW_2009_8_28_10_50_DEF

#pragma   warning(disable:4786)
#pragma   warning(disable:4251)
#pragma   warning(disable:4089)
#pragma   warning(disable:4996)
#pragma   warning(disable:4099)

#include <math.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <io.h>
#include <string>
#include <vector>
#include <algorithm> 
#include <functional> 

using namespace::std;

//Message Define
//////////////////////////////////////////////////////////////////////////
#ifndef     WM_PROCESS_MSG
#define		WM_PROCESS_MSG		WM_USER + 2012
enum PROCESSMSG{	
	PROC_START =   10,
	PROC_STEP  =   11,
	PROC_OVER  =   12,	
	PROC_PRINT =   13,
};
#endif
//////////////////////////////////////////////////////////////////////////

//Math Define
//////////////////////////////////////////////////////////////////////////
#define BMP_HEADER_MARKER  ((WORD) ('M'<<8)|'B')
#define FOURBYTES(bits)    (((bits)+31)/32*4)

#ifndef PI
#define PI         3.1415926535897932384626433832795
#endif

#ifndef R2D
#define R2D        (180.0/PI)
#define R2D400     (200.0/PI)
#endif

#ifndef D2R
#define D2R        (PI/180.0)
#define D2R400     (PI/200.0)
#endif
//////////////////////////////////////////////////////////////////////////

//File Define
//////////////////////////////////////////////////////////////////////////
#ifndef PATHLEN
#define PATHLEN  512
#endif

#ifndef NAMELEN
#define NAMELEN  128
#endif

#ifndef IDLEN
#define IDLEN    16
#endif
//////////////////////////////////////////////////////////////////////////

//Point Define
//////////////////////////////////////////////////////////////////////////
#ifndef  _PT2F
#define  _PT2F
typedef struct tagPT2F
{ 	
	float x,y;
}PT2F,*PPT2F;
#endif

#ifndef  _PT2FID
#define  _PT2FID
typedef struct tagPT2FID
{ 	
	char  strID[IDLEN];
	float x,y;
}PT2FID,*PPT2FID;
#endif

#ifndef _PT2D
#define _PT2D
typedef struct tagPT2D
{ 	
	double x,y;
}PT2D,*PPT2D;
#endif

#ifndef _PT2DID
#define _PT2DID
typedef struct tagPT2DID
{ 	
	char  strID[IDLEN];
	double x,y;
}PT2DID,*PPT2DID;
#endif

#ifndef _PT3F
#define _PT3F
typedef struct tagPT3F
{ 	
	float x,y,z;
}PT3F,*PPT3F;
#endif

#ifndef _PT3FID
#define _PT3FID
typedef struct tagPT3FID
{ 	
	char  strID[IDLEN];
	float x,y,z;
}PT3FID,*PPT3FID;
#endif

#ifndef _PT3D
#define _PT3D
typedef struct tagPT3D
{ 	
	double x,y,z;
}PT3D,*PPT3D;
#endif

#ifndef _PT3DID
#define _PT3DID
typedef struct tagPT3DID
{ 	
	char  strID[IDLEN];
	double x,y,z;
}PT3DID,*PPT3DID;
#endif
//////////////////////////////////////////////////////////////////////////

//Object Define
//////////////////////////////////////////////////////////////////////////
#ifndef  _OBJECTBOX
#define  _OBJECTBOX
typedef struct tagRECTBOX
{ 	
	int    nFlag;
	float  MinX,MaxX;
	float  MinY,MaxY;
}RECTBOX,*PRECTBOX;
typedef struct tagPOLYGON
{ 	
	int    nFlag;
	float *px,*py;
	int    nNum;
}POLYGON,*PPOLYGON;
typedef struct tagPOLYGON3D
{ 	
	int    nFlag;
	float *px,*py,*pz;
	int    nNum;
}POLYGON3D,*PPOLYGON3D;
typedef struct tagCUBEBOX
{ 	
	int    nFlag;
	float  MinX,MaxX;
	float  MinY,MaxY;
	float  MinZ,MaxZ;
}CUBEBOX,*PCUBEBOX;
typedef struct tagSPHEREBOX
{ 	
	float fCx,fCy,fCZ;
	float fRadius;
}SPHEREBOX,*PSPHEREBOX; 
#endif
//////////////////////////////////////////////////////////////////////////

#endif // BasicDef_h__