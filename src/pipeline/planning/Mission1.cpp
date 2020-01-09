#include "Mission1.h"
#include "best_theta/BestThetaFinder.h"
#include "../DebugImage.h"


using namespace std;
using namespace student;

/**
 * Execute the planning of Mission 1.
 * @return if planning is succesfull, it returns a list of DubisCurve that solve the mission
 */
boost::optional<vector<DubinsCurve>> Mission1::solve() {
    sortVictimsByTheirNumber();
    addRobotVictimsAndGateToCleanestPathsGraph();
    computeShortestPath();

    // draw shortest path for debugging purposes
    DebugImage::drawPath(shortestPath);
    DebugImage::showAndWait();

    // since Votonoi generated a lot of points, we prune the shortest path, removing close points
    prunePath(&shortestPath, toReach, 0.075);

    // draw pruned shortest path for debugging purposes
    DebugImage::drawPath(shortestPath, cv::Scalar(150, 150, 150));
    DebugImage::showAndWait();

    // we execute best_thet to find the best list of not-colliding DubinsCurves
    BestThetaFinder finder(10, start.theta, collisionDetector);
    return finder.findBestDubinsCurves(shortestPath);
}

/**
 * Add the points of the robot. the victims and the gate to the cleanest path graph (so that later
 * we can execute Dijkstra)
 */
void Mission1::addRobotVictimsAndGateToCleanestPathsGraph() {
    toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(Point(start.x, start.y)));
    for (const auto &victim:sortedVictims) {
        Point *copyOfVictim = cleanestPaths->addAndConnectToNearestPoint(victim);
        toReach.push_back(copyOfVictim);
    }
    toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(gate));
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
void Mission1::computeShortestPath() {
    for (int i = 0; i < toReach.size() - 1; i++) {
        vector<Point *> path = cleanestPaths->shortestPathFromTo(toReach[i], toReach[i + 1]);

        // skip the first path point if this is not the path that start from the start position (to avoid
        // duplicates on the list of paths)
        if (i == 0) {
            shortestPath.insert(shortestPath.end(), path.begin(), path.end());
        } else {
            shortestPath.insert(shortestPath.end(), path.begin() + 1, path.end());
        }
    }
}


void Mission1::sortVictimsByTheirNumber() {
    vector<pair<int, Point>> sortedVictimsWithIndex = victims;
    sort(sortedVictimsWithIndex.begin(), sortedVictimsWithIndex.end(), [](pair<int, Point> &a, pair<int, Point> &b) {
        return a.first < b.first;
    });
    for (const auto &victimWithIndex:sortedVictimsWithIndex) {
        sortedVictims.push_back(victimWithIndex.second);
    }
}


