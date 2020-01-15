#ifndef STUDENT_PROECT_FIND_GATE_HPP
#define STUDENT_PROECT_FIND_GATE_HPP

#include <utility>

#include "ShapeDetector.h"
namespace student {

    class GateDetector : public ShapeDetector<Polygon> {
    protected:
        cv::Mat applyColorMask(const cv::Mat &hsvImage) override;

        vector<vector<cv::Point>> filterContours(const vector<vector<cv::Point>> &contours) override;

        vector<Polygon> mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contours,
                                   const cv::Mat &hsvImage, const cv::Mat &filteredImage) override;

    public:
        GateDetector() : ShapeDetector(3) {}
    };
}


#endif //STUDENT_PROECT_FIND_GATE_HPP
