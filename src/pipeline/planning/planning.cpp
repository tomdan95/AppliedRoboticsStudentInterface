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

    vector<Point *> addPointsToReach(Graph *cleanestPaths,
                                     Point start,
                                     vector<Point> victims,
                                     Polygon gate) {
        vector<Point *> toReach;
        toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(start));

        for (const auto &victim:victims) {
            Point *copyOfVictim = cleanestPaths->addAndConnectToNearestPoint(victim); // TODO: Only one neighbor??
            toReach.push_back(copyOfVictim);
        }
        toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(getPolygonCenter(gate)));
        return toReach;
    }


    vector<Point *>
    computeShortestPath(Graph *cleanestPaths, vector<Point *> toReach) {
        vector<Point *> allPaths;
        for (int i = 0; i < toReach.size() - 1; i++) {
            vector<Point *> path = cleanestPaths->shortestPathFromTo(toReach[i], toReach[i + 1]);
            if (i == 0) {
                allPaths.insert(allPaths.end(), path.begin(), path.end());
            }else{
                allPaths.insert(allPaths.end(), path.begin() + 1, path.end());
            }

        }
        return allPaths;
    }


    bool canSkip(vector<Point *> toReach, vector<Point *>::iterator it) {
        for (auto &i : toReach) {
            if (i == *it) {
                return false;
            }
        }
        return true;
    }

    void prunePath(vector<Point *> *path, vector<Point *> toReach) {
        cout << "PATH = " << path->size() << endl;
        auto it = path->begin();
        while (it + 1 != path->end()) {

            double distance = Graph::distanceBetween(**it, **(it + 1));
            bool isShort = distance < 0.15;
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

    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {
        vector<Polygon> inflatedObstacles = inflateObstacles(obstacleList, borders);
        Graph cleanestPaths = findCleanestPaths(inflatedObstacles, obstacleList);// TODO: OBSTACLES NOT INFLATED!!!


        vector<Point *> toReach = addPointsToReach(&cleanestPaths, Point(x, y), getSortedVictimPoints(victimList),
                                                   gate);


        DebugImage::clear();
        DebugImage::drawGraph(cleanestPaths);
        for (auto *p:toReach) {
            DebugImage::drawPoint(*p);
        }

        vector<Point *> shortestPath = computeShortestPath(&cleanestPaths, toReach);

        DebugImage::drawPath(shortestPath);
        DebugImage::showAndWait();

        prunePath(&shortestPath, toReach);

        DebugImage::drawPath(shortestPath, cv::Scalar(200, 200, 100));
        DebugImage::showAndWait();


        vector<DubinsCurve> curves = findBestTheta(shortestPath,
                                                   theta); // TODO: add obstacles to also do collision checking

        cout << "discetizing" << endl;
        vector<Pose> allPoses;
        for (auto curve:curves) {
            vector<Pose> poses = dubinsCurveToPoseVector(curve);
            allPoses.insert(allPoses.end(), poses.begin(), poses.end());
        }
        cout << "done" << endl;
        path.setPoints(allPoses);

        DebugImage::clear();
        DebugImage::drawGraph(cleanestPaths);
        DebugImage::drawPoses(allPoses);
        DebugImage::drawPath(shortestPath, cv::Scalar());
        DebugImage::showAndWait();

        return true;
    }


    // TODO: Split sorting and center
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


}