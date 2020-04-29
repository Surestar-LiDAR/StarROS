/**
 * @author   lucb
 * @date     2020/2/14
 */

#ifndef __STAR_SDK_FMT_RGP_H
#define __STAR_SDK_FMT_RGP_H

#include <star/Star.h>
#include <star/fmt/Writer.h>
#include <vector>
#include <star/Lidar.h>
#include <star/Configure.h>

#include <Rec3SDK.h>

namespace ss {
namespace fmt {

class __star_export RgpWriter : public FileWriter {
public:
    RgpWriter();

    void setup(const Configure& configure) override;

    void open(const std::string& path) override;
    void close() override;

    std::size_t write(const Point& point) override;
    std::size_t writeLines();


    std::string suffix() const override;

protected:
    struct RGPPoint {
        float    xyz[3];
        uint32_t intensity;
    };

//    struct Point {
//        float    xyz[3];
//        uint32_t intensity;
//    };

    using RGPLine = std::vector<RGPPoint>;

private:
    RGPLine                 _line;
    std::vector<RGPLine>    _lines;
    double                  _lastAngle;
    std::size_t             _lastIndex;
    bool                    _deleteLine ;          //删除扫描线
    double                  _angleRes;

    std::size_t             _totalCount;
    uint32_t                _rows;
    uint32_t                _columns;

    Rec_PointCloudPtr       _resultCloud;
    Rec_PointFlags          _validFlag;
    Rec_PointFlags          _invalidFlag;
    float*                  _reflectanceBuffer;
    std::string             _filePath;

    constexpr static int RGP_DELETE_LINE_COUNT = 2 ;  //起始时删除扫描线的个数
    constexpr static int RGP_DELETE_POINT_COUNT = 100 ; //一条扫描线最少点的个数
};
}
}

#endif //__STAR_SDK_FMT_RGP_H
