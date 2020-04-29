/**
 * @author   lucb
 * @date     2020/1/10
 */

#ifndef __STAR_SDK_FMT_XYZ_H
#define __STAR_SDK_FMT_XYZ_H

#include <star/fmt/Writer.h>
#include "Point.h"

namespace ss {
namespace fmt {

class __star_export XyzTxtWriter : public FileWriter {
public:
    typedef enum {
        XYZ_WRITE_DEFAULT,
        XYZ_WRITE_UTC,
        XYZ_WRITE_TZERO,
        XYZ_WRITE_ANGLE_RANGE
    } XyzTxtWriteType;

     XyzTxtWriter() noexcept;
    explicit XyzTxtWriter(const std::string& path) noexcept;

//    explicit XyzTxtWriter(std::ostream& os) noexcept;

    void setWriteType(XyzTxtWriteType type);

    std::string suffix() const override;

    size_t write(const Point& point) override;

    void close() override;

private:
    XyzTxtWriteType _type;
};

}
}

#endif //__STAR_SDK_FMT_XYZ_H
