#define FIND_ROBOT_DEBUG_PLOT
#include "../opencv-utils.h"
#include "student_image_elab_interface.hpp"
#include "../pipeline/detection/find_victims.hpp"

using namespace std;
using namespace student;

int main() {
    cv::Mat img = loadImage("/home/lar2019/robot/AppliedRoboticsStudentInterface/src/areana_samples/000.jpg");
    cv::Mat hsv = convertRGBToHSV(img);

    string configFolder = "/tmp";
    Config config(configFolder);
    VictimDetector detector(img, config);
    auto victims = detector.findPolygons(hsv, 256);
    
    cout << "Found " << victims.size() << " victims" << endl;

    return 0;
}
