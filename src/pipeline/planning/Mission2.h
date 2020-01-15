#ifndef STUDENT_PROJECT_MISSION2_H
#define STUDENT_PROJECT_MISSION2_H

#include "MissionSolver.h"

using namespace std;

namespace student {
    class Mission2 : public MissionSolver {
    private:
        const int victimBonus;
    public:
        Mission2(const CollisionDetector *collisionDetector, Graph *cleanestPaths, const RobotPosition &start,
                 const Point &gate, const vector<pair<int, Point>> &victims,double pruneThreshold, int victimBonus) : MissionSolver(
                collisionDetector, cleanestPaths, start, gate, victims, pruneThreshold), victimBonus(victimBonus) {}

        boost::optional<vector<DubinsCurve>> solve() override;
    };
}


#endif
