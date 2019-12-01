#ifndef STUDENT_PROECT_FIND_VICTIMS_HPP
#define STUDENT_PROECT_FIND_VICTIMS_HPP

#include <utility>

#include "ShapeDetector.h"

namespace student {

    class Victim{
    public:
        const Polygon polygon;
        const int number;

        Victim(Polygon polygon, const int number) : polygon(std::move(polygon)), number(number) {};
    };

    class VictimDetector : public ShapeDetector<Victim> {
    protected:
        cv::Mat applyColorMask(const cv::Mat &hsvImage) override;

        vector<vector<cv::Point>> filterContours(const vector<vector<cv::Point>> &contours) override;

        vector<Victim> mapPolygons(vector<Polygon> polygons) override;

    public:
        VictimDetector() : ShapeDetector(1) {}
    };
}


#endif //STUDENT_PROECT_FIND_VICTIMS_HPP
