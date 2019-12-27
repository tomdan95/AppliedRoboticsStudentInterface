/**
 * To execute this function, set 'planning' to false.
 */

#include "planning.h"
#include <chrono>
#include "clipper/clipper.hpp"
#include "../utils.h"

#include "voronoi/voronoi_cleanest_path.h"
#include "Graph.h"
#include "../../opencv-utils.h"
#include "../DebugImage.h"
#include "best_theta/best_theta.h"

#define INT_ROUND 1000.0

using namespace std;

namespace student {

    void
    connectStartAndGateAndVictimsToCleanestPathsGraph(const vector<pair<int, Polygon>> &victims, const Polygon &gate,
                                                      Point start, Graph *cleanestPaths);

    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {
        vector<Polygon> inflatedObstacles = inflateObstacles(obstacleList, borders);
        Graph cleanestPaths = findCleanestPaths(inflatedObstacles, obstacleList);// TODO: OBSTACLES NOT INFLATED!!!
        connectStartAndGateAndVictimsToCleanestPathsGraph(victimList, gate, Point(x, y), &cleanestPaths);

        // from start to gate
        Point *start = cleanestPaths.addAndConnectToNearestPoint(Point(x, y));
        Point *copyOfGate = cleanestPaths.addAndConnectToNearestPoint(getPolygonCenter(victimList[3].second));


        DebugImage::drawGraph(cleanestPaths);

        DebugImage::drawPoint(*start);
        DebugImage::drawPoint(*copyOfGate);

        vector<Point*> shortestPath = cleanestPaths.shortestPathFromTo(start, copyOfGate);

        DebugImage::drawPath(shortestPath);


        vector<DubinsCurve> curves = findBestTheta(shortestPath, theta); // TODO: add obstacles to also do collision checking

        vector<Pose> poses;
        for(auto curve:curves) {
            dubinsCurveToPoseVector(curve, poses);
        }
        path.setPoints(poses);

        DebugImage::drawPoses(poses);
        DebugImage::showAndWait();

        return true;
    }

    void
    connectStartAndGateAndVictimsToCleanestPathsGraph(const vector<pair<int, Polygon>> &victims, const Polygon &gate,
                                                      Point start, Graph *cleanestPaths) {
        //cleanestPaths->connectTo(start);
        //cleanestPaths->connectTo(getPolygonCenter(gate));
        for (const auto &victim:victims) {
            cleanestPaths->addAndConnectToNearestPoint(getPolygonCenter(victim.second));
        }
    }

    vector<Point> getSortedVictimPoints(const vector<pair<int, Polygon>> &victimList) {
        vector<pair<int, Polygon>> sortedVictimList = victimList;
        sort(sortedVictimList.begin(), sortedVictimList.end(), [](pair<int, Polygon> &a, pair<int, Polygon> &b) {
            return a.first < b.first;
        });
        vector<Point> victimPoints;
        for (const pair<int, Polygon> &victim:sortedVictimList) {
            Point victimCenter = getPolygonCenter(victim.second);
            victimPoints.push_back(victimCenter);
        }
        return victimPoints;
    }

    // TODO: Move to a separate file
    vector<Polygon> inflateObstacles(const vector<Polygon> &obstacles, const Polygon &borders) {
        vector<Polygon> returnObstacles;
        ClipperLib::Clipper cl, clFinal;
        ClipperLib::Paths MergedObstacles, FinalArena;
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

            co.Execute(clipperInflatedObstacle, 30); // TODO: Change the offset value according to the robot size
            cl.AddPaths(clipperInflatedObstacle, ClipperLib::ptSubject, true);
        }
        cl.Execute(ClipperLib::ctUnion, MergedObstacles, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        ClipperLib::Path clipperBorders;
        for (const auto &point : borders) {
            clipperBorders << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
        }

        clFinal.AddPath(clipperBorders, ClipperLib::ptSubject, true);
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


    // TODO: Move to dubis folder
    vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve, vector<Pose> &vector) {
        dubinsArcToPoseVector(curve.a1, vector);
        dubinsArcToPoseVector(curve.a2, vector);
        dubinsArcToPoseVector(curve.a3, vector);
        return vector;
    }


    void dubinsArcToPoseVector(DubinsArc arc, vector<Pose> &vector) {
        const int numPoints = 20;
        for (int i = 0; i < numPoints; i++) {
            DubinsArc temp;
            double s = arc.L / numPoints * ((float) i);
            circleLine(s, arc.x0, arc.y0, arc.th0, arc.k, &temp);
            // TODO: temp.k doesn't get updated. Do we need it in this data structure??
            vector.emplace_back(1, temp.xf, temp.yf, temp.thf, arc.k);
        }
    }

}