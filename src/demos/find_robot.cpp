#define FIND_ROBOT_DEBUG_PLOT
#include "../pipeline/detection/find_robot.hpp"
#include "../opencv-utils.h"


int main() {
    cv::Mat img = loadImage("/home/lar2019/robot/AppliedRoboticsStudentInterface/src/areana_samples/000.jpg");
    Polygon triangle;
    double baricenterX, baricenterY;
    double theta;
    bool found = student::findRobot(img, 512, triangle, baricenterX, baricenterY, theta, "/tmp");

    if (found) {
        cout << "Found robot!" << endl;
        cout << "x: " << baricenterX << " y: " << baricenterY << " theta: " << theta << endl;
    } else {
        cout << "Robot not found!" << endl;
        return -1;
    }


    return 0;
}

