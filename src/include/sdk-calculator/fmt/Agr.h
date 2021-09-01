/**
 * @author   lucb
 * @date     2020/2/14
 */

#ifndef __STAR_SDK_AGR_H
#define __STAR_SDK_AGR_H

#include <sdk-decoder/Star.h>

#include <sdk-calculator/fmt/Writer.h>

namespace ss {
namespace fmt {

class AgrTxtWriter : public FileWriter {
public:
    AgrTxtWriter();

    void writeHeader() override;

    size_t write(const Point& point) override;

    std::string suffix() const override;
};

class AgrBinWriter : public FileWriter {
public:
    AgrBinWriter();

    void writeHeader() override;
    size_t write(const Point& point) override;

    std::string suffix() const override;
};

}
}

#endif //__STAR_SDK_AGR_H
