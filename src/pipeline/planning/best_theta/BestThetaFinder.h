#ifndef STUDENT_PROJECT_BEST_THETA_H
#define STUDENT_PROJECT_BEST_THETA_H

#include <utils.hpp>
#include "../collision_detection/CollisionDetector.h"
#include "../dubins/models.h"

using namespace std;

namespace student {

    class BestThetaFinder {
    private:
        const CollisionDetector *collisionDetector;
        bool findShortestNotCollidingCurve(const RobotPosition &start, const RobotPosition &end, DubinsCurve &shortestCurve);
        bool findShortestNotCollidingCurve(DubinsCurve &shortestCurve, double thetaStart, double thetaEnd, const Point &start, const Point &end);
    public:
        explicit BestThetaFinder(const CollisionDetector *collisionDetector) : collisionDetector(collisionDetector) {}

        vector<DubinsCurve> findBestDubinsCurves(const vector<Point *> &path, double robotTheta);
    };

}

#endif
