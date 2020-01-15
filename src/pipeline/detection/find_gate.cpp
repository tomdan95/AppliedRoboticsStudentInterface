#include "find_gate.hpp"
#include "../../opencv-utils.h"


namespace student {
    cv::Mat GateDetector::applyColorMask(const cv::Mat &hsvImage) {
        // Find green regions

        cv::Mat green_mask;
        
        //cv::inRange(hsvImage, cv::Scalar(60, 45, 40), cv::Scalar(80, 215, 180), green_mask);
        cv::inRange(hsvImage, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);
        //showImageAndWait(green_mask);

        return green_mask;
    }

    vector<vector<cv::Point>> GateDetector::filterContours(const vector<vector<cv::Point>> &contours) {
        vector<vector<cv::Point>> filtered;
        for (auto &contour : contours) {
            double area = cv::contourArea(contour);
            cout << "[FIND-GATE] contours = " << contour.size() << endl;
            if (contour.size() == 4 || contour.size() == 5) {
                filtered.push_back(contour);
            }
            /*
            if (area > 500 && area < 5000) {
                filtered.push_back(contour);
            }*/
        }
        return filtered;
    }

    vector<Polygon> student::GateDetector::mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contours,
                                         const cv::Mat &hsvImage, const cv::Mat &filteredImage) {
        return polygons;
    }

}
