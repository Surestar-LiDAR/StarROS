/**
 * @author   lucb
 * @date     2020/2/20
 */

#ifndef __STAR_SDK_FMT_PLR_H
#define __STAR_SDK_FMT_PLR_H

#include <star/Star.h>
#include <star/fmt/Writer.h>

namespace ss {
namespace fmt {

class __star_export PlrTxtWriter : public FileWriter {
public:
    PlrTxtWriter();

    void writeHeader() override;

    size_t write(const Point& point) override;

    std::string suffix() const override;

};

class __star_export PlrBinWriter : public FileWriter {
public:
    PlrBinWriter();

    void writeHeader() override;

    size_t write(const Point& point) override;

    std::string suffix() const override;
};

}
}

#endif //__STAR_SDK_FMT_PLR_H
