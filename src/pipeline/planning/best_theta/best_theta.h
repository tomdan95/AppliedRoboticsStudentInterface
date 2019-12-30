#ifndef STUDENT_PROJECT_BEST_THETA_H
#define STUDENT_PROJECT_BEST_THETA_H

#include <utils.hpp>
#include "../collision_detection/CollisionDetector.h"
#include "../dubins/models.h"

using namespace std;

namespace student {
    vector<DubinsCurve>
    findBestDubinsCurves(const vector<Point>& path, double robotTheta, CollisionDetector *collisionDetector);
}

#endif
