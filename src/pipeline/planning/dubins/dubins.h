#ifndef STUDENT_PROJECT_DUBINS_H
#define STUDENT_PROJECT_DUBINS_H


#include <iostream>
#include <cmath>
#include "models.h"
#include "primitives.h"
#include <vector>
#include "student_planning_interface.hpp"

using namespace std;
using namespace student;


vector<DubinsCurve> dubinsShortestPath(RobotPosition start, RobotPosition end, double kMax);

void dubinsArcToPoseVector(DubinsArc arc, vector<Pose> &dst);
vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve);

#endif
