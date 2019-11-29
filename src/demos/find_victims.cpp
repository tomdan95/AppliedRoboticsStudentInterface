#define FIND_ROBOT_DEBUG_PLOT
#include "../pipeline/detection/find_robot.hpp"
#include "../pipeline/detection/utils.h"
#include "student_image_elab_interface.hpp"
#include "../pipeline/detection/find_victims.hpp"

using namespace std;
using namespace student;

cv::Mat loadImage(string fileName);
void showImageAndWaitKeyPress(cv::Mat &image);

int main() {
    cv::Mat img = loadImage("/home/lar2019/robot/AppliedRoboticsStudentInterface/src/areana_samples/000.jpg");
    cv::Mat hsv = convertRGBToHSV(img);
    
    VictimDetector detector;
    auto victims = detector.findPolygons(hsv, 2);
    
    cout << "Found " << victims.size() << " victims" << endl;

    return 0;
}

cv::Mat loadImage(string fileName) {
    cv::Mat image = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);
    if (!image.data) {
        cout << "Could not open or find the image: " << fileName << endl;
        cout << "(If you're using a relative file path, check from which folder you're running this executable)" << endl;
        exit(-1);
    }
    return image;
}


void showImageAndWaitKeyPress(cv::Mat &image) {
    namedWindow("Loaded image", cv::WINDOW_AUTOSIZE);// Create a window for display.
    imshow("Display window", image);
    cv::waitKey(0);
}
