#ifndef STUDENT_PROJECT_SHADOW_COLLISION_DETECTOR_H
#define STUDENT_PROJECT_SHADOW_COLLISION_DETECTOR_H

#include <opencv2/core/mat.hpp>
#include "CollisionDetector.h"

using namespace std;

namespace student {

    class ShadowCollisionDetector : public CollisionDetector {
    private:
        cv::Mat obstaclesShadow;

    public:
        explicit ShadowCollisionDetector(const Polygon &borders, const vector<Polygon> &obstacles, const Polygon &gate, const vector<Polygon> &victims);

        bool isPointInAnyObstacle(const Point &point) const override;

        bool doesSegmentCollide(const Point &a, const Point &b) const override;

        bool doesCurveCollide(const DubinsCurve &curve) const override;
    };
}


#endif
