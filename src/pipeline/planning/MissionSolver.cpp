#include "MissionSolver.h"

using namespace std;
using namespace student;

bool canSkip(vector<Point *> toReach, Point* p) {
    for (auto &i : toReach) {
        if (i == p) {
            return false;
        }
    }
    return true;
}


void MissionSolver::prunePath(vector<Point *> *path, vector<Point *> toReach, double threshold) {
    cout << "PATH = " << path->size() << endl;
    auto it = path->begin();
    while (it + 1 != path->end()) {
        Point* p = *it;
        Point* next = *(it + 1);

        // TODO: This may add collisions, that then doublin can't fix!
        // TODO: maybe, don't compute euclidian distances but path distances
        double distance = Graph::distanceBetween(*p, *next);
        cout << distance << endl;
        bool isShort = distance < threshold;
        if (isShort && canSkip(toReach, p) && !collisionDetector->doesSegmentCollide(**(it - 1), *next)) {
            it = path->erase(it);
        } else if (isShort && canSkip(toReach, next)) { // TODO: Check collision also here
            path->erase(it + 1);
        } else {
            it++;
        }
    }
    cout << "PATH AFTER = " << path->size() << endl;
}


