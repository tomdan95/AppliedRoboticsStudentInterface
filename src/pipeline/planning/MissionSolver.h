#ifndef STUDENT_PROJECT_MISSIONSOLVER_H
#define STUDENT_PROJECT_MISSIONSOLVER_H

#include <boost/optional.hpp>
#include "Graph.h"
#include "dubins/models.h"
#include "../detection/find_obstacles.hpp"
#include "collision_detection/CollisionDetector.h"


using namespace std;

namespace student {
    class MissionSolver {
    protected:
        const CollisionDetector *collisionDetector;
        Graph *cleanestPaths;
        const RobotPosition start;
        const Point gate;
        const vector<pair<int, Point>> victims;

        void prunePath(vector<Point *> *path, vector<Point *> toReach);


    public:
        MissionSolver(const CollisionDetector *collisionDetector, Graph *cleanestPaths,
                      const RobotPosition &start,
                      const Point &gate,
                      const vector<pair<int, Point>> victims) : collisionDetector(collisionDetector),
                                                                   cleanestPaths(cleanestPaths),
                                                                   start(start), gate(gate),
                                                                   victims(victims) {}

        virtual boost::optional<vector<DubinsCurve>> solve() = 0;


    };
}

#endif //STUDENT_PROJECT_MISSIONSOLVER_H
