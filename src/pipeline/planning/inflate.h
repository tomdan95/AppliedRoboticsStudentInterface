#ifndef STUDENT_PROJECT_INFLATE_H
#define STUDENT_PROJECT_INFLATE_H

#include "student_planning_interface.hpp"

vector<Polygon> inflateObstacles(const vector<Polygon> &obstacles, double robotSize);
vector<Polygon> deflateArenaBorders(const Polygon &borders, double robotSize);
vector<Polygon> resizeObstaclesAndBorders(const vector<Polygon> &obstacles, const Polygon &borders, double robotSize);

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "dubins/dubins.h"
#include "../detection/find_robot.hpp"

#endif //STUDENT_PROJECT_INFLATE_H
