#include "MissionSolver.h"

using namespace std;
using namespace student;

bool canSkip(vector<Point *> toReach, Point *p) {
    for (auto &i : toReach) {
        if (i == p) {
            return false;
        }
    }
    return true;
}


void MissionSolver::prunePath(vector<Point *> *path, vector<Point *> toReach, double threshold) {
    cout << "[PATH-PRUNING] Original path nodes = " << path->size() << endl;
    auto it = path->begin() + 1;
    while (it + 1 != path->end()) {
        auto previous = it - 1;
        auto p = it;
        auto next = it + 1;

        double distance = Graph::distanceBetween(**p, **next);
        bool isShort = distance < threshold;
        bool removed = false;
        if (isShort && canSkip(toReach, *p) && !collisionDetector->doesSegmentCollide(**previous, **next)) {
            // we don't need to reach point 'p', and segment from 'previous' to 'next' doesn't have
            // collisions, so we can remove p
            it = path->erase(it);
            removed = true;
        } else if (isShort && canSkip(toReach, *next)) {
            auto nextNext = next + 1;
            if (nextNext != path->end() && !collisionDetector->doesSegmentCollide(**p, **nextNext)) {
                // we don't need to reach the next point, and segment from point 'p' to the successor of next
                // doesn't have collisions, so we ca remove it
                path->erase(next);
                removed = true;
            }
        }
        if (!removed) {
            it++;
        }

    }
    cout << "[PATH-PRUNING] PATH POINTS AFTER = " << path->size() << endl;
}


/**
 * Add the points of the robot. the victims and the gate to the cleanest path graph (so that later
 * we can execute Dijkstra)
 */
void MissionSolver::addRobotVictimsAndGateToCleanestPathsGraph(const vector<Point>& sortedVictims) {
    toReach.push_back(cleanestPaths->addAndConnectToNearNotCollidingPoints(Point(start.x, start.y), collisionDetector));
    for (const auto &victim:sortedVictims) {
        Point *copyOfVictim = cleanestPaths->addAndConnectToNearNotCollidingPoints(victim, collisionDetector);
        toReach.push_back(copyOfVictim);
    }
    toReach.push_back(cleanestPaths->addAndConnectToNearNotCollidingPoints(gate, collisionDetector));
}

/**
 * Computes the shortest path frm the robot position to the gate, passing for each victim.
 * We execute Dijkstra from:
 * - start position to victim 1
 * - victim 1 to victim 2
 * ...
 * - victim n-1 to victim n
 * - victim n to gate
 */
void MissionSolver::computeShortestPath() {
    vector<Point *> lastGeneratedPath;
    for (int i = 0; i < toReach.size() - 1; i++) {
        vector<Point *> path = cleanestPaths->shortestPathFromTo(toReach[i], toReach[i + 1], lastGeneratedPath);

        // skip the first path point if this is not the path that start from the start position (to avoid
        // duplicates on the list of paths)
        if (i == 0) {
            shortestPath.insert(shortestPath.end(), path.begin(), path.end());
        } else {
            shortestPath.insert(shortestPath.end(), path.begin() + 1, path.end());
        }
        lastGeneratedPath = path;
    }
}
