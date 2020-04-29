/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
#ifndef ICD_COMMON_H
#define ICD_COMMON_H

//#define TRUE 1
//#define FALSE 0
#include <string.h>
#include <stdlib.h>
#include "publicfile.h"
#pragma pack(2)

static int getRevNbr(const char* svnRev) {
  char strNbr[10] = "0\0";
  unsigned int count = 0;
  unsigned int nbrFlag = 2;

  for(unsigned int i = 0; i < strlen(svnRev); i++) {
    if ( nbrFlag == 0 ) {
      if ( svnRev[i] == ' ') {
    strNbr[count] = '\0';
    break;
      }
      else  {
    strNbr[count++] = svnRev[i];
      }
    }
    else {
      if ( svnRev[i] == ' ')  {
    nbrFlag--;
      }
    }
  }
  return atoi(strNbr);
}

#ifdef _WINDOWS
//typedef long           int_32;
//typedef long           int32;
//typedef long           time_ld;
//typedef unsigned long  uint_32;
typedef short          int_16;
typedef short          int16;
typedef unsigned short uint_16;
typedef unsigned char  BYTE;
typedef unsigned char  uchar;

typedef unsigned int            uint_32;
typedef int                     int_32;
typedef long  long               int_64;
typedef unsigned long  long         uint_64;
#else
typedef unsigned int            uint_32;
typedef int                     int_32;
typedef long                    int_64;
typedef unsigned long           uint_64;
#endif

typedef struct
{
    int msgid;
    int hour, minute, second;
    int day_num, month, year;
    int julian;
    int seconds;   //zks(13/01/11): change time_t to int for standard
    int numBeats;  // count of heartbeat msgs
} DateMsg_S;

#pragma pack()


#endif
