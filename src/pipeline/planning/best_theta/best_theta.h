#ifndef STUDENT_PROJECT_BEST_THETA_H
#define STUDENT_PROJECT_BEST_THETA_H

#include <utils.hpp>
#include "../dubins/curve.h"
#include "../collision_detection/CollisionDetector.h"

using namespace std;

namespace student {
    vector<DubinsCurve>
    findBestDubinsCurves(vector<Point *> path, double robotTheta, CollisionDetector *collisionDetector);
}

#endif
