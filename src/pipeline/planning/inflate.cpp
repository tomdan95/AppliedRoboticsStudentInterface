#include "best_theta/best_theta.h"
#include "../DebugImage.h"
#include "../../opencv-utils.h"
#include "Graph.h"
#include "voronoi/voronoi_cleanest_path.h"
#include "../utils.h"
#include "clipper/clipper.hpp"
#include <chrono>
#include "planning.h"
#include "inflate.h"

#define INT_ROUND 1000.0

vector<Polygon> inflateObstacles(const vector<Polygon> &obstacles, const Polygon &borders, int robotSize) {
    vector<Polygon> returnObstacles;
    ClipperLib::Clipper cl, clFinal;
    ClipperLib::Paths MergedObstacles, clipperInflatedBoarders, FinalArena;
    for (const Polygon &obstacle : obstacles) {
        ClipperLib::Path clipperObstacle;
        ClipperLib::Paths clipperInflatedObstacle;
        for (const auto &point : obstacle) {
            clipperObstacle << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
        }
        //clipperObstacle << ClipperLib::IntPoint(obstacle[0].x * INT_ROUND, obstacle[0].y * INT_ROUND);

        ClipperLib::ClipperOffset co;
        cout << "inflating, size " << obstacle.size() << endl;
        co.AddPath(clipperObstacle, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);

        co.Execute(clipperInflatedObstacle, robotSize); // TODO: Change the offset value according to the robot size
        cl.AddPaths(clipperInflatedObstacle, ClipperLib::ptSubject, true);
    }
    cl.Execute(ClipperLib::ctUnion, MergedObstacles, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    ClipperLib::Path clipperBorders;
    for (const auto &point : borders) {
        clipperBorders << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
    }

    ClipperLib::ClipperOffset bor;

    bor.AddPath(clipperBorders, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    bor.Execute(clipperInflatedBoarders, -robotSize);

    clFinal.AddPaths(clipperInflatedBoarders, ClipperLib::ptSubject, true);
    clFinal.AddPaths(MergedObstacles, ClipperLib::ptClip, true);
    clFinal.Execute(ClipperLib::ctDifference, FinalArena, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
    for (const auto &mergedPath : FinalArena) {
        Polygon mergedObstacle;
        for (const auto &point : mergedPath) {
            mergedObstacle.emplace_back(point.X / INT_ROUND, point.Y / INT_ROUND);
        }
        //inflatedObstacle.erase()
        returnObstacles.push_back(mergedObstacle);
    }
    return returnObstacles;
}