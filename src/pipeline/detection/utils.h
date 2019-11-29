#ifndef STUDENT_PROECT_UTILS_H
#define STUDENT_PROECT_UTILS_H

#include "student_image_elab_interface.hpp"

using namespace std;

namespace student {
    cv::Mat convertRGBToHSV(const cv::Mat &rgb);

    vector<cv::Point> getPointsFromPolygon(Polygon &polygon);
}

#endif //STUDENT_PROECT_UTILS_H
