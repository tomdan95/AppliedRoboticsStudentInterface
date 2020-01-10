#include "find_gate.hpp"
#include "../../opencv-utils.h"


namespace student {
    cv::Mat GateDetector::applyColorMask(const cv::Mat &hsvImage) {
        // Find green regions
        cv::Mat green_mask;
        cv::inRange(hsvImage, cv::Scalar(40, 40, 50), cv::Scalar(75, 255, 255), green_mask);

        showImageAndWaitKeyPress(green_mask);


        return green_mask;
    }

    vector<vector<cv::Point>> GateDetector::filterContours(const vector<vector<cv::Point>> &contours) {
        vector<vector<cv::Point>> filtered;
        for (auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500 && area<5000) {
                filtered.push_back(contour);
            }
        }
        return filtered;
    }

    vector<Polygon> student::GateDetector::mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contours,
                                         const cv::Mat &hsvImage, const cv::Mat &filteredImage) {
        return polygons;
    }

}
