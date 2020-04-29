/**
 * @author   lucb
 * @date     2020/2/14
 */

#ifndef __STAR_SDK_YSJ_H
#define __STAR_SDK_YSJ_H

#include <star/Star.h>
#include <star/fmt/Writer.h>

namespace ss {
namespace fmt {
class __star_export YsjWriter : public FileWriter {
public:
    YsjWriter();
    size_t write(const Point& point) override;

    std::string suffix() const override;
};

}
}

#endif //__STAR_SDK_YSJ_H
