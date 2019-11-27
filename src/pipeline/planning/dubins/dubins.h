#ifndef STUDENT_PROECT_DUBINS_H
#define STUDENT_PROECT_DUBINS_H


#include <iostream>
#include <cmath>
#include "scale.h"
#include "temp.h"
#include "primitives.h"
#include "curve.h"


DubinsCurve dubinsShortestPath(RobotPosition start, RobotPosition end, double kMax);

#endif
