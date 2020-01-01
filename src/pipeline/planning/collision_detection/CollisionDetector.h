#ifndef STUDENT_PROJECT_COLLISIONDETECTOR_H
#define STUDENT_PROJECT_COLLISIONDETECTOR_H

#include <utility>
#include <utils.hpp>
#include <opencv2/core/mat.hpp>
#include "../dubins/models.h"

using namespace std;

namespace student {

    class CollisionDetector {
    private:
        cv::Mat obstaclesShadow;

    public:
        explicit CollisionDetector(const Polygon &borders, const vector<Polygon>& obstacles);

        bool isPointInAnyObstacle(const Point &point) const;

        bool doesCurveCollide(const DubinsCurve &curve) const;
    };
}


#endif
