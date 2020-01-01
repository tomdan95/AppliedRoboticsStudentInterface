#include "best_theta/BestThetaFinder.h"
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

ClipperLib::Paths processClipperObstacles(const vector<Polygon> &obstacles, int robotSize){
    ClipperLib::Clipper cl;
    ClipperLib::Paths MergedObstacles;

    for (const Polygon &obstacle : obstacles) {
        ClipperLib::Path clipperObstacle;
        ClipperLib::Paths clipperInflatedObstacle;
        for (const auto &point : obstacle) {
            clipperObstacle << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
        }

        ClipperLib::ClipperOffset co;
        co.AddPath(clipperObstacle, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        co.Execute(clipperInflatedObstacle, robotSize);

        cl.AddPaths(clipperInflatedObstacle, ClipperLib::ptSubject, true);
    }
    cl.Execute(ClipperLib::ctUnion, MergedObstacles, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return MergedObstacles;
}

ClipperLib::Paths processClipperBorders(const Polygon &borders, int robotSize){
    ClipperLib::ClipperOffset bor;
    ClipperLib::Path clipperBorders;
    ClipperLib::Paths clipperDeflatedBoarders;

    for (const auto &point : borders) {
        clipperBorders << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
    }

    bor.AddPath(clipperBorders, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    bor.Execute(clipperDeflatedBoarders, -robotSize);
    return clipperDeflatedBoarders;
}

vector<Polygon> inflateObstacles(const vector<Polygon> &obstacles, int robotSize){
    vector<Polygon> returnObstacles;
    ClipperLib::Paths clipperObstacles = processClipperObstacles(obstacles, robotSize);
    for (const auto &clipperObstacle : clipperObstacles) {
        Polygon Obstacle;
        for (const auto &point : clipperObstacle) {
            Obstacle.emplace_back(point.X / INT_ROUND, point.Y / INT_ROUND);
        }
        returnObstacles.push_back(Obstacle);
    }
    return returnObstacles;
}

vector<Polygon> deflateArenaBorders(const Polygon &borders, int robotSize){
    vector<Polygon> returnBorders;
    ClipperLib::Paths clipperBorders = processClipperBorders(borders, robotSize);
    for (const auto &clipperBorder : clipperBorders) {
        Polygon border;
        for (const auto &point : clipperBorder) {
            border.emplace_back(point.X / INT_ROUND, point.Y / INT_ROUND);
        }
        returnBorders.push_back(border);
    }
    return returnBorders;
}

vector<Polygon> resizeObstaclesAndBorders(const vector<Polygon> &obstacles, const Polygon &borders, int robotSize){
    vector<Polygon> returnArena;
    ClipperLib::Clipper clFinal;
    ClipperLib::Paths arena;
    ClipperLib::Paths clipperObstacles = processClipperObstacles(obstacles, robotSize);
    ClipperLib::Paths clipperBorders = processClipperBorders(borders, robotSize);

    clFinal.AddPaths(clipperBorders, ClipperLib::ptSubject, true);
    clFinal.AddPaths(clipperObstacles, ClipperLib::ptClip, true);
    clFinal.Execute(ClipperLib::ctDifference, arena, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
    for (const auto &arenaPath : arena){
        Polygon arenaComponent;
        for (const auto &point : arenaPath) {
            arenaComponent.emplace_back(point.X / INT_ROUND, point.Y / INT_ROUND);
        }
        returnArena.push_back(arenaComponent);
    }
    return returnArena;
}