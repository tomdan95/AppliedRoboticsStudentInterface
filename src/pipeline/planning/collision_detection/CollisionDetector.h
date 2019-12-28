#ifndef STUDENT_PROJECT_COLLISIONDETECTOR_H
#define STUDENT_PROJECT_COLLISIONDETECTOR_H

#include <utility>
#include <utils.hpp>
#include <opencv2/core/mat.hpp>
#include "../dubins/curve.h"

#define OBSTACLES_MATRIX_SIDE 1500

using namespace std;

namespace student {

    class CollisionDetector {
    private:
        cv::Mat obstaclesShadow;
        static bool isPointInAnyObstacle(const Point &point, vector<Polygon> obstacles);
        static int isPointInObstacle(Point p, const Polygon& polygon);


    public:
        explicit CollisionDetector(vector<Polygon> obstacles);

        bool isPointInAnyObstacle(const Point &point);

        bool doesCurveCollide(DubinsCurve curve);
    };
}


#endif
