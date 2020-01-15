#ifndef STUDENT_PROJECT_MISSION2_H
#define STUDENT_PROJECT_MISSION2_H

#include "MissionSolver.h"

using namespace std;

namespace student {
    class Mission2 : public MissionSolver {
    private:
        const int victimBonus;

        vector<vector<pair<int, Point>>> generatePermutations();

    public:
        Mission2(const CollisionDetector *collisionDetector, Graph *cleanestPaths, const RobotPosition &start,
                 const Point &gate, const vector<pair<int, Point>> &victims, Config &config) : MissionSolver(
                collisionDetector, cleanestPaths, start, gate, victims, config), victimBonus(config.getVictimBonus()) {}

        boost::optional<vector<DubinsCurve>> solve() override;
    };
}


#endif
