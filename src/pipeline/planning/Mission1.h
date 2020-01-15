#ifndef STUDENT_PROJECT_MISSION1_H
#define STUDENT_PROJECT_MISSION1_H

#include "MissionSolver.h"

using namespace std;

namespace student {

    class Mission1 : public MissionSolver {
    public:
        Mission1(const student::CollisionDetector *collisionDetector, student::Graph *cleanestPaths,
                 const RobotPosition &start, const Point &gate, const vector<pair<int, Point>>& victims, double pruneThreshold)
                : MissionSolver(collisionDetector, cleanestPaths, start, gate, victims, pruneThreshold) { }

        boost::optional<vector<DubinsCurve>> solve() override;

    private:
        vector<Point> sortedVictims;

        void sortVictimsByTheirNumber();
    };

}

#endif //STUDENT_PROJECT_MISSION1_H
