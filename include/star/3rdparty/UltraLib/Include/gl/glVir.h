/********************************************************************
       Author:		           QQ	
       File Name:	           glVir
	   File Type:	           h
       Created Date:	       2008/12/14
	   Created Time:	       18:19      
*********************************************************************/
#ifndef glVir_h_ZQW_2008_12_14_18_19_DEF
#define glVir_h_ZQW_2008_12_14_18_19_DEF

#pragma   warning(disable:4786)
#pragma   warning(disable:4251)
#pragma   warning(disable:4089)
#pragma   warning(disable:4996)
#pragma   warning(disable:4099)

#include <io.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <algorithm> 
#include <functional>  
using namespace::std;

#include "gl.h"
#include "glu.h"
#include "glext.h"
#include "wglext.h"
#include "gl3.h"

#pragma comment( lib,"opengl32.lib" )
#pragma message( "Automatically linking with opengl32.lib" )
#pragma comment( lib,"glu32.lib" )
#pragma message( "Automatically linking with glu32.lib" )

#ifndef _GLVECTOR
#define _GLVECTOR
typedef struct tagVECT2F
{ 
	GLfloat  x;
	GLfloat  y;
}VECT2F,*PVECT2F;

typedef struct tagVECT3F
{ 
	GLfloat  x;
	GLfloat  y;
	GLfloat  z;
}VECT3F,*PVECT3F;

typedef struct tagVECT2D
{ 
	GLdouble  x;
	GLdouble  y;
}VECT2D,*PVECT2D;

typedef struct tagVECT3D
{ 
	GLdouble  x;
	GLdouble  y;
	GLdouble  z;
}VECT3D,*PVECT3D;

#endif

#ifndef _GLPARA
#define _GLPARA
typedef struct tagBORDPAN
{ 
	GLfloat  xNear,xfar;
	GLfloat  yNear,yfar;
	GLfloat  zNear,zfar;
}BORDPAN,*PBORDPAN;

typedef struct tagVIEWPORT
{ 
	GLdouble StartX,StartY;
	GLdouble Width,Height;
}VIEWPORT,*PVIEWPORT;

typedef struct tagFRUSTUMDIST
{ 
	GLdouble Left,Right;
	GLdouble Bottom,Top;
	GLdouble zNear,zFar;
}FRUSTUMDIST,*PFRUSTUMDIST;

typedef struct tagFRUSTUMANGLE
{ 
	GLdouble vAgl;
	GLdouble hAgl;
	GLdouble whRatio;
	GLdouble ScrPlanRatio;
	GLdouble Near;
	GLdouble Far;
}FRUSTUMANGLE,*PFRUSTUMANGLE;

typedef struct tagORTHPARA
{ 
	GLdouble Left,Right;
	GLdouble Bottom,Top;
	GLdouble Near,Far;	
	GLdouble Ratio;
}ORTHPARA,*PORTHPARA;

#endif

#endif // glVir_h__