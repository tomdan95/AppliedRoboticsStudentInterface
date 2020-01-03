#include "MissionSolver.h"

using namespace std;
using namespace student;

bool canSkip(vector<Point *> toReach, vector<Point *>::iterator it) {
    for (auto &i : toReach) {
        if (i == *it) {
            return false;
        }
    }
    return true;
}


void MissionSolver::prunePath(vector<Point *> *path, vector<Point *> toReach) {
    cout << "PATH = " << path->size() << endl;
    auto it = path->begin();
    while (it + 1 != path->end()) {
        // TODO: This may add collisions, that then doublin can't fix!
        // TODO: maybe, don't compute euclidian distances but path distances
        double distance = Graph::distanceBetween(**it, **(it + 1));
        bool isShort = distance < 0.075; // 7,5cm
        if (isShort && canSkip(toReach, it)) {
            it = path->erase(it);
        } else if (isShort && canSkip(toReach, it + 1)) {
            path->erase(it + 1);
        } else {
            it++;
        }
    }
    cout << "PATH AFTER = " << path->size() << endl;
}


