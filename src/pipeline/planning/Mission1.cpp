#include "Mission1.h"
#include "best_theta/best_theta.h"


using namespace std;
using namespace student;

vector<DubinsCurve> Mission1::solve() {
    addPointsToReach();
    computeShortestPath();
    prunePath(&shortestPath, toReach);
    return findBestDubinsCurves(shortestPath, start.theta, collisionDetector);
}

void Mission1::addPointsToReach() {
    toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(Point(start.x, start.y)));
    for (const auto &victim:sortedVictims) {
        Point *copyOfVictim = cleanestPaths->addAndConnectToNearestPoint(victim);
        toReach.push_back(copyOfVictim);
    }
    toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(gate));
}


void Mission1::computeShortestPath() {
    for (int i = 0; i < toReach.size() - 1; i++) {
        vector<Point *> path = cleanestPaths->shortestPathFromTo(toReach[i], toReach[i + 1]);
        if (i == 0) {
            shortestPath.insert(shortestPath.end(), path.begin(), path.end());
        } else {
            shortestPath.insert(shortestPath.end(), path.begin() + 1, path.end());
        }
    }
}
