#ifndef STUDENT_PROECT_FIND_OBSTACLES_HPP
#define STUDENT_PROECT_FIND_OBSTACLES_HPP

#include <utility>

#include "ShapeDetector.h"
namespace student {

    class ObstacleDetector : public ShapeDetector<Polygon> {
    protected:
        cv::Mat applyColorMask(const cv::Mat &hsvImage) override;

        vector<Polygon> mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contours,
                                   const cv::Mat &hsvImage, const cv::Mat &filteredImage) override;

    public:
        ObstacleDetector() : ShapeDetector(10) {}
    };
}


#endif //STUDENT_PROECT_FIND_VICTIMS_HPP
