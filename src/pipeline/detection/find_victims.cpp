#include "find_victims.hpp"
#include "../../opencv-utils.h"


namespace student {
    cv::Mat VictimDetector::applyColorMask(const cv::Mat &hsvImage) {
        cv::Mat greenMask;
        // works on simulator
        //cv::inRange(hsvImage, cv::Scalar(45, 50, 50), cv::Scalar(75, 255, 255), greenMask);
        
        // works on real arena
        //cv::inRange(hsvImage, cv::Scalar(40, 40, 50), cv::Scalar(75, 230, 160), greenMask);

        // works in simulator
        // TODO: Check if it also works on the real arena
        cv::inRange(hsvImage, cv::Scalar(40, 40, 50), cv::Scalar(75, 255, 255), greenMask);
        //showImageAndWaitKeyPress(greenMask);

        return greenMask;
    }

    vector<vector<cv::Point>> VictimDetector::filterContours(const vector<vector<cv::Point>> &contours) {
        vector<vector<cv::Point>> filtered;
        for (auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 5000) {
                filtered.push_back(contour);
            }
        }
        return filtered;
    }

    vector<Victim>
    student::VictimDetector::mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contours,
                                         const cv::Mat &hsvImage, const cv::Mat &filteredImage) {
        vector<cv::Rect> rects;
        for (const auto &contour:contours) {
            rects.push_back(boundingRect(cv::Mat(contour)));
        }


        vector<int> digits = digitClassifier.recognizeDigits(rgbImage, hsvImage, filteredImage, rects);
        vector<Victim> victims;
        for (vector<Polygon>::size_type i = 0; i < polygons.size(); i++) {
            victims.emplace_back(polygons[i], digits[i]);
        }
        return victims;
    }

}
