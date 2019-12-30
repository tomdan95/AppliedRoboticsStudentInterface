#ifndef STUDENT_PROJECT_MISSION1_H
#define STUDENT_PROJECT_MISSION1_H

#include "MissionSolver.h"

using namespace std;

namespace student {

    class Mission1 : public MissionSolver {
    public:
        Mission1(const student::CollisionDetector *collisionDetector, student::Graph *cleanestPaths,
                 const RobotPosition &start, const Point &gate, const vector<Point> &sortedVictims)
                : MissionSolver(collisionDetector, cleanestPaths, start, gate, sortedVictims) {}

        vector<DubinsCurve> solve() override;

    private:
        vector<Point *> toReach;
        vector<Point *> shortestPath;

        void addPointsToReach();

        void computeShortestPath();
    };

}

#endif //STUDENT_PROJECT_MISSION1_H
