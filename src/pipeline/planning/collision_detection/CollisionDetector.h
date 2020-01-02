#ifndef STUDENT_PROJECT_COLLISION_DETECTOR_H
#define STUDENT_PROJECT_COLLISION_DETECTOR_H

#include <utility>
#include <utils.hpp>
#include "../dubins/models.h"

using namespace std;

namespace student {

    class CollisionDetector {

    public:
        virtual bool isPointInAnyObstacle(const Point &point) const = 0;
        virtual bool doesCurveCollide(const DubinsCurve &curve) const = 0;
    };
}


#endif
