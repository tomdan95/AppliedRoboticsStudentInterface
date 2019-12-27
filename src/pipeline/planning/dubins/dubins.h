#ifndef STUDENT_PROECT_DUBINS_H
#define STUDENT_PROECT_DUBINS_H


#include <iostream>
#include <cmath>
#include "scale.h"
#include "temp.h"
#include "primitives.h"
#include "curve.h"
#include <vector>
#include "student_planning_interface.hpp"

using namespace std;
using namespace student;

DubinsCurve dubinsShortestPath(RobotPosition start, RobotPosition end, double kMax);

void dubinsArcToPoseVector(DubinsArc arc, vector<Pose> &dst);
vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve);

#endif
