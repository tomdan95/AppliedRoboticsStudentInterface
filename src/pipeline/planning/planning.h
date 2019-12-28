
#ifndef STUDENT_PROECT_PLANNING_H
#define STUDENT_PROECT_PLANNING_H

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "dubins/temp.h"
#include "dubins/dubins.h"
#include "dubins/curve.h"
#include "../detection/find_robot.hpp"


using namespace std;

namespace student {
    vector<Point> getSortedVictimPoints(const vector<pair<int, Polygon>> &victimList);

    void dubinsArcToPoseVector(DubinsArc arc, vector<Pose>& vector);
    vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve, vector<Pose> &poseVector);
}

#endif //STUDENT_PROECT_PLANNING_H
