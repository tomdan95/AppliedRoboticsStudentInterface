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
    addRobotVictimsAndGateToCleanestPathsGraph(sortedVictims);
    computeShortestPath();

    // draw shortest path for debugging purposes
    DebugImage::drawGraph(*cleanestPaths);
    DebugImage::drawPath(shortestPath);
    DebugImage::showAndWait();

    // since Voronoi generated a lot of points, we prune the shortest path, removing close points
    prunePath(&shortestPath, toReach, pruneThreshold);

    // draw pruned shortest path for debugging purposes
    DebugImage::drawPath(shortestPath, cv::Scalar(150, 150, 150));
    DebugImage::showAndWait();

    // we execute best_theta to find the best list of not-colliding DubinsCurves
    return finder.findBestDubinsCurves(shortestPath);
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


