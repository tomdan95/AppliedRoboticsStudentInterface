#ifndef STUDENT_PROECT_FIND_ROBOT_HPP
#define STUDENT_PROECT_FIND_ROBOT_HPP

#include "student_image_elab_interface.hpp"

using namespace std;

namespace student {

    void convertColorsFromRGBToHSV(const cv::Mat &img_in, const cv::Mat &hsv_img);

    vector<vector<cv::Point>> findBlueContours(const cv::Mat &hsv_img);

}
#endif //STUDENT_PROECT_FIND_ROBOT_HPP
