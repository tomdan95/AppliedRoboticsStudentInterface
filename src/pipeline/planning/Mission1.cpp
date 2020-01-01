#include "Mission1.h"
#include "best_theta/BestThetaFinder.h"
#include "../DebugImage.h"


using namespace std;
using namespace student;

boost::optional<vector<DubinsCurve>> Mission1::solve() {
    sortVictims();
    addPointsToReach();
    computeShortestPath();
    prunePath(&shortestPath, toReach);
    BestThetaFinder finder(10, start.theta, collisionDetector);// TODO: Move maxK
    return finder.findBestDubinsCurves(shortestPath);
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


void Mission1::sortVictims() {
    vector<pair<int, Point>> sortedVictimsWithIndex = victims;
    sort(sortedVictimsWithIndex.begin(), sortedVictimsWithIndex.end(), [](pair<int, Point> &a, pair<int, Point> &b) {
        return a.first < b.first;
    });
    for (const auto &victimWithIndex:sortedVictimsWithIndex) {
        sortedVictims.push_back(victimWithIndex.second);
    }
}


