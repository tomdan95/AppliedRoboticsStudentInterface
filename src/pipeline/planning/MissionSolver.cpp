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


