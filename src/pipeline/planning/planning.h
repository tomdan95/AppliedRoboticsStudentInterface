
#ifndef STUDENT_PROECT_PLANNING_H
#define STUDENT_PROECT_PLANNING_H

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "dubins/dubins.h"
#include "../detection/find_robot.hpp"


using namespace std;

namespace student {
    vector<pair<int, Point>> getVictimPoints(const vector<pair<int, Polygon>> &victims);

    void fixS(vector<Pose> &poses);

    vector<Pose> discretizeListOfDubinsCurves(const vector<DubinsCurve> &curves);

}

#endif //STUDENT_PROECT_PLANNING_H
