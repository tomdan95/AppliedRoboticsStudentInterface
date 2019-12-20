#define FIND_ROBOT_DEBUG_PLOT

#include "../opencv-utils.h"
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

using namespace std;
using namespace student;

vector<Polygon> generateObstacles() {
    vector<Polygon> obstacles;

    Polygon square;
    square.emplace_back(0.2, 0.2);
    square.emplace_back(0.2, 0.3);
    square.emplace_back(0.3, 0.3);
    square.emplace_back(0.3, 0.2);
    //obstacles.push_back(square);


    Polygon triangle;
    triangle.emplace_back(0.5, 0.5);
    triangle.emplace_back(0.6, 0.5);
    triangle.emplace_back(0.55, 0.55);
    obstacles.push_back(triangle);

    return obstacles;
}

vector<Point> generateGate(){
    vector<Point> gate;
    gate.emplace_back(0.9, 0.9);
    return gate;
}

int main() {
    cv::Mat img = loadImage("/home/lar2019/robot/AppliedRoboticsStudentInterface/src/areana_samples/000.jpg");
    cv::Mat hsv = convertRGBToHSV(img);

    vector<Point> borders; // TODO: fill
    auto obstacles = generateObstacles();
    vector<pair<int, vector<Point>>> victims;
    auto gate = generateGate();
    Path path;
    planPath(borders, obstacles, victims, gate, 0.1, 0.1, 1, path, "");

    return 0;
}
