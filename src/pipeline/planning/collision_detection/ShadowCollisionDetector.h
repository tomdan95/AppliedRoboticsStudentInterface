#ifndef STUDENT_PROJECT_SHADOW_COLLISION_DETECTOR_H
#define STUDENT_PROJECT_SHADOW_COLLISION_DETECTOR_H

#include <opencv2/core/mat.hpp>
#include "CollisionDetector.h"

using namespace std;

namespace student {

    class ShadowCollisionDetector:public CollisionDetector {
    private:
        cv::Mat obstaclesShadow;

    public:
        explicit ShadowCollisionDetector(const Polygon &borders, const vector<Polygon>& obstacles);

        bool isPointInAnyObstacle(const Point &point) const override;

        bool doesCurveCollide(const DubinsCurve &curve) const override;
    };
}


#endif
