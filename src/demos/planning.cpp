#define FIND_ROBOT_DEBUG_PLOT

#include "../opencv-utils.h"
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

using namespace std;
using namespace student;


int main() {
    cv::Mat img = loadImage("/home/lar2019/robot/AppliedRoboticsStudentInterface/src/areana_samples/000.jpg");
    cv::Mat hsv = convertRGBToHSV(img);

    vector<Point> robotTriangle;
    double x, y, theta;
    findRobot(img, 1, robotTriangle, x, y, theta, "");


    vector<Polygon> obstacles;
    vector<pair<int, Polygon>> victims;
    Polygon gate;
    processMap(img, 1, obstacles, victims, gate, "");


    vector<Point> borders;

    Path path;
    planPath(borders, obstacles, victims, gate, 0.1, 0.1, 1, path, "");

    return 0;
}
