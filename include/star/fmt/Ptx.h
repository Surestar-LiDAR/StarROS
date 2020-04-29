/**
 * @author   lucb
 * @date     2020/2/20
 */

#ifndef __STAR_SDK_FMT_PTX_H
#define __STAR_SDK_FMT_PTX_H

#include <star/Star.h>
#include <star/fmt/Writer.h>

namespace ss {
namespace fmt {

class __star_export PtxWriter : public FileWriter {
public:
    PtxWriter();

    std::string suffix() const override;

    size_t write(const Point& point) override;

    void open(const std::string& path) override;

protected:
    void matrixCmp();
    bool statisticLines(const Point& point);
    void writeHeader(std::ofstream& os);

    struct PtxPoint {
        double x;
        double y;
        double z;
        double i;
        double a;
        double t;
    };
private:
    double _delta[3];
    double _l2g[3];
    double _l2gm[3][3];
    double _b0;
    uint32_t _writeCount;
    uint32_t _zwriteCount;
    uint32_t _scanLineNum[2];
    uint32_t _linePointNum;
    double   _lastAngle;

    std::vector<PtxPoint> _points;
    std::vector<PtxPoint> _zpoints;
    std::string    _zosFilePath;
    std::ofstream  _zos;
};

}
}



#endif //__STAR_SDK_FMT_PTX_H
