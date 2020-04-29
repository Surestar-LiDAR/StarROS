/**
 * @author   lucb
 * @date     2020/2/20
 */

#ifndef __STAR_SDK_FMT_ST_H
#define __STAR_SDK_FMT_ST_H

#include <star/Star.h>
#include <star/fmt/Writer.h>

#include <star/Configure.h>

namespace ss {
namespace fmt {

class __star_export StWriter : public FileWriter {
public:
    StWriter();

    void setup(const Configure& configure) override;

    std::string suffix() const override;

    size_t write(const Point& point) override;

protected:
    cfg::CommonCalibParams _calibParams;
    cfg::ApCmpCoor         _apCmpCoor;
    std::ofstream          _ofs[4];
    uint32_t               _tzero;
    double                 _second;
    double                 _left_time;
};

}
}


#endif //__STAR_SDK_FMT_ST_H
