#ifndef STUDENT_PROECT_FIND_ROBOT_HPP
#define STUDENT_PROECT_FIND_ROBOT_HPP

#include "student_image_elab_interface.hpp"

using namespace std;

namespace student {

    // TODO: make private
    void convertColorsFromRGBToHSV(const cv::Mat &img_in, const cv::Mat &hsv_img);

    // TODO: make private
    vector<vector<cv::Point>> findBlueContours(const cv::Mat &hsv_img);

    // TODO: make private
    bool findRobotTriangle(vector<std::vector<cv::Point>> &contours, vector<vector<cv::Point>> &contoursApprox,
                           vector<cv::Point> &approxCurve, cv::Mat &contoursImg);
}
#endif //STUDENT_PROECT_FIND_ROBOT_HPP
