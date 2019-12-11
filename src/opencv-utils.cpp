

#include "opencv-utils.h"

using namespace std;

cv::Mat convertRGBToHSV(const cv::Mat &rgb) {
    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
    return hsv;
}

cv::Mat loadImage(string fileName) {
    cv::Mat image = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);
    if (!image.data) {
        cout << "Could not open or find the image: " << fileName << endl;
        cout << "(If you're using a relative file path, check from which folder you're running this executable)"
             << endl;
        exit(-1);
    }
    return image;
}


void showImageAndWaitKeyPress(const cv::Mat &image) {
    namedWindow("Loaded image", cv::WINDOW_AUTOSIZE);// Create a window for display.
    imshow("Display window", image);
    cv::waitKey(0);
}


cv::Mat rotateImage(const cv::Mat &image, int degrees) {
    cv::Point2f src_center(image.cols / 2.0F, image.rows / 2.0F);
    cv::Mat rot_mat = getRotationMatrix2D(src_center, degrees, 1.0);
    cv::Mat rotated_image;
    warpAffine(image, rotated_image, rot_mat, image.size());
    return rotated_image;
}
