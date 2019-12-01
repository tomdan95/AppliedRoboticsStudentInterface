#ifndef STUDENT_PROECT_FIND_VICTIMS_HPP
#define STUDENT_PROECT_FIND_VICTIMS_HPP

#include <utility>

#include "ShapeDetector.h"
#include "digit_classification/DigitClassifier.h"

namespace student {

    class Victim {
    public:
        const Polygon polygon;
        const int number;

        Victim(Polygon polygon, const int number) : polygon(std::move(polygon)), number(number) {};
    };

    class VictimDetector : public ShapeDetector<Victim> {
    private:
        const DigitClassifier digitClassifier;
    protected:
        cv::Mat applyColorMask(const cv::Mat &hsvImage) override;

        vector<vector<cv::Point>> filterContours(const vector<vector<cv::Point>> &contours) override;

        vector<Victim> mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contours,
                                   const cv::Mat &hsvImage, const cv::Mat &filteredImage) override;

    public:
        VictimDetector() : ShapeDetector(10), digitClassifier() {}
    };
}


#endif //STUDENT_PROECT_FIND_VICTIMS_HPP
