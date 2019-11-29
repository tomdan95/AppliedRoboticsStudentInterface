#ifndef STUDENT_PROECT_FIND_VICTIMS_HPP
#define STUDENT_PROECT_FIND_VICTIMS_HPP

#include "ShapeDetector.h"

namespace student {

    class Victim{
    public:
        const Polygon polygon;
        const int number;

        Victim(const Polygon &polygon, const int number) : polygon(polygon), number(number) {};
    };

    class VictimDetector : public ShapeDetector<Victim> {
    protected:
        cv::Mat applyColorMask(cv::Mat &hsvImage) override;

        vector<Victim> mapPolygons(vector<Polygon> polygons) override;

    public:
        VictimDetector() : ShapeDetector(1) {}
    };
}


#endif //STUDENT_PROECT_FIND_VICTIMS_HPP
