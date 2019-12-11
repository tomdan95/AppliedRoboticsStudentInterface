#include "find_obstacles.hpp"
#include "../../opencv-utils.h"


namespace student {
    cv::Mat ObstacleDetector::applyColorMask(const cv::Mat &hsvImage) {
        cv::Mat red_mask_low, red_mask_high, red_mask;
        cv::inRange(hsvImage, cv::Scalar(0, 10, 10), cv::Scalar(15, 255, 255), red_mask_low);
        cv::inRange(hsvImage, cv::Scalar(175, 10, 10), cv::Scalar(179, 255, 255), red_mask_high);
        cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);

        //showImageAndWaitKeyPress(red_mask);


        return red_mask;
    }

    vector<Polygon> student::ObstacleDetector::mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contours,
                                         const cv::Mat &hsvImage, const cv::Mat &filteredImage) {
        return polygons;
    }

}
