/**
 * @author   lucb
 * @date     2020/2/14
 */

#ifndef __STAR_SDK_RFANS_H
#define __STAR_SDK_RFANS_H

#include <star/Star.h>
#include <star/fmt/Writer.h>
#include <star/fmt/Xyz.h>

namespace ss {
namespace fmt {

class __star_export RFansWriter : public FileWriter {
public:
    RFansWriter();

    void open(const std::string& path) override;

    size_t write(const Point& point) override;

    std::string suffix() const override;

private:
    constexpr static std::size_t OUTPUT_FILE_COUNT = (32);
    XyzTxtWriter _xyzs[OUTPUT_FILE_COUNT];
};

}
}

#endif //__STAR_SDK_RFANS_H
