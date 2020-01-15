#ifndef STUDENT_PROJECT_MISSIONSOLVER_H
#define STUDENT_PROJECT_MISSIONSOLVER_H

#include <boost/optional.hpp>
#include "Graph.h"
#include "dubins/models.h"
#include "../detection/find_obstacles.hpp"
#include "collision_detection/CollisionDetector.h"
#include "best_theta/BestThetaFinder.h"
#include "../Config.h"


using namespace std;

namespace student {

    /**
     * MissionSolver represents the logic to solve a mission.
     * It's an abstract class implemented by Mission1 and Mission2
     */
    class MissionSolver {
    protected:
        const CollisionDetector *collisionDetector;
        Graph *cleanestPaths;
        const RobotPosition start;
        const Point gate;
        const vector<pair<int, Point>> victims;
        vector<Point *> toReach;
        vector<Point *> shortestPath;
        const double pruneThreshold;
        BestThetaFinder finder;

        void addRobotVictimsAndGateToCleanestPathsGraph(const vector<Point>& sortedVictims);

        void computeShortestPath();

        void prunePath(vector<Point *> *path, vector<Point *> toReach, double threshold);


    public:
        MissionSolver(const CollisionDetector *collisionDetector, Graph *cleanestPaths,
                      const RobotPosition &start,
                      const Point &gate,
                      const vector<pair<int, Point>> victims,
                      Config& config) : collisionDetector(collisionDetector),
                                                                   cleanestPaths(cleanestPaths),
                                                                   start(start), gate(gate),
                                                                   victims(victims),
                                                                   pruneThreshold(config.getPruneThreshold()),
                                                                   finder(config.getBestThetaSteps(), config.getMaxK(), start.theta, collisionDetector) {}

        virtual boost::optional<vector<DubinsCurve>> solve() = 0;

    };
}

#endif //STUDENT_PROJECT_MISSIONSOLVER_H
