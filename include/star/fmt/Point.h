/**
 * @author   lucb
 * @date     2020/1/8
 */

#ifndef __STAR_SDK_FMT_POINTER_H
#define __STAR_SDK_FMT_POINTER_H

#include <star/Star.h>
#include <star/Shot.h>
#include <cstdint>
#include <cstddef>

namespace ss {
namespace fmt {

struct Color {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

struct Point {
    //Las data
    double   x;
    double   y;
    double   z;
    uint16_t intensity;
    uint8_t  returnNumber; //echo number start from 0
    uint8_t  returnCount;
    uint8_t  scanDirectionFlag;
    uint8_t  edgeOfFlightLine;
    uint8_t  classification;
    int8_t   scanAngleRank;
    uint8_t  userData;
    uint16_t sourceId;
    double   gpsTimestamp;
    Color    color;

    //Xyz text data for AngleRange
    double   turnAngle;
    double   angle;
    double   range;
    uint32_t tzero;
    double   pulseWidth;
    uint32_t riseEdge;
    int      mpiaSel;
    double   stageAngle;
    double   bubbleX;
    double   bubbleY;
    int      mirrorNumber;

    bool set(const SHOTS_CALCOUT_S& shot, std::size_t idx);
};

}
}


#endif //__STAR_SDK_FMT_POINTER_H
