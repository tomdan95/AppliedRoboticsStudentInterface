#ifndef STUDENT_PROJECT_OPENCV_UTILS_H
#define STUDENT_PROJECT_OPENCV_UTILS_H

#include "student_image_elab_interface.hpp"

using namespace std;


cv::Mat convertRGBToHSV(const cv::Mat &rgb);

cv::Mat loadImage(string fileName);

cv::Mat rotateImage(const cv::Mat &image, int degrees);

void showImageAndWait(const cv::Mat &image, int wait = 0, const string& name = "debug");

#endif //STUDENT_PROECT_OPENCV_UTILS_H
