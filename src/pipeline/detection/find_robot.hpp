#ifndef STUDENT_PROECT_FIND_ROBOT_HPP
#define STUDENT_PROECT_FIND_ROBOT_HPP

#include "student_image_elab_interface.hpp"
#include "ShapeDetector.h"

using namespace std;

namespace student {



    class RobotDetector:public ShapeDetector {
    private:
        cv::Mat applyColorMask(cv::Mat &hsvImage) override;

        vector<Polygon> filterPolygons(vector<Polygon> polygons) override;
    };

    // TODO: make private
    cv::Mat getBlueMask(const cv::Mat &hsv);

    // TODO: make private
    void convertColorsFromRGBToHSV(const cv::Mat &img_in, const cv::Mat &hsv_img);

    // TODO: make private
    vector<vector<cv::Point>> findBlueContours(const cv::Mat &hsv_img);

    // TODO: make private
    bool findRobotTriangle(vector<std::vector<cv::Point>> &contours, vector<vector<cv::Point>> &contoursApprox,
                           vector<cv::Point> &approxCurve, cv::Mat &contoursImg);
}
#endif //STUDENT_PROECT_FIND_ROBOT_HPP
