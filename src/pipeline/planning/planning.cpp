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
#include "inflate.h"
#include "Scaler.h"

using namespace std;

namespace student {

    vector<Point *>
    addPointsToReachToCleanestPaths(Graph *cleanestPaths, Point start, vector<Point> victims, Polygon gate) {
        vector<Point *> toReach;
        toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(start));
        for (const auto &victim:victims) {
            toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(victim));
        }
        toReach.push_back(cleanestPaths->addAndConnectToNearestPoint(getPolygonCenter(gate)));
        return toReach;
    }


    vector<Point *>
    computeShortestPath(Graph *cleanestPaths, vector<Point *> toReach) {
        vector<Point *> completePath;
        for (int i = 0; i < toReach.size() - 1; i++) {
            vector<Point *> subPath = cleanestPaths->shortestPathFromTo(toReach[i], toReach[i + 1]);
            if (i == 0) {
                completePath.insert(completePath.end(), subPath.begin(), subPath.end());
            } else {
                // don't insert two times the same path
                completePath.insert(completePath.end(), subPath.begin() + 1, subPath.end());
            }

        }
        return completePath;
    }


    bool canSkip(vector<Point *> toReach, vector<Point *>::iterator it) {
        for (auto &i : toReach) {
            if (i == *it) {
                return false;
            }
        }
        return true;
    }

    void prunePath(vector<Point *> *path, const vector<Point *>& toReach) {
        cout << "[PATH-PRUNING] Complete path has " << path->size() << " points" << endl;
        auto it = path->begin();
        while (it + 1 != path->end()) {
            // TODO: This may add collisions, that then doublin can't fix!
            // TODO: maybe, don't compute euclidian distances but path distances
            double distance = Graph::distanceBetween(**it, **(it + 1));
            bool isShort = distance < (0.1 * 800.0); // 10cm // TODO: Depends on scale!!
            if (isShort && canSkip(toReach, it)) {
                it = path->erase(it);
            } else if (isShort && canSkip(toReach, it + 1)) {
                path->erase(it + 1);
            } else {
                it++;
            }
        }
        cout << "[PATH-PRUNING] Path pruned, now it has " << path->size() << " points" << endl;
    }

    vector<Point> toPointVector(const vector<Point *>& pointers) {
        vector<Point> points;
        for(const auto *pointerToPoint:pointers){
            points.push_back(*pointerToPoint);
        }
        return points;
    }

    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {

        auto t_start = std::chrono::high_resolution_clock::now();

        auto sortedVictims = getSortedVictimPoints(victimList);

        Scaler scaler(800.0);
        auto scaledArenaBorders = scaler.scaleToInt(borders);
        auto scaledObstacles = scaler.scaleToInt(obstacleList);
        auto scaledVictims = scaler.scaleToInt(sortedVictims);
        auto scaledGate = scaler.scaleToInt(gate);
        auto scaledRobot = scaler.scaleToInt(Point(x, y));



        vector<Polygon> inflatedObstacles = inflateObstacles(scaledObstacles, scaledArenaBorders, 10); //TODO:put correct robot size


        CollisionDetector detector(scaledObstacles);// TODO: Not inflated!!

        Graph cleanestPaths = findCleanestPaths(inflatedObstacles, &detector);// TODO: OBSTACLES NOT INFLATED!!!
        vector<Point *> toReach = addPointsToReachToCleanestPaths(&cleanestPaths, scaledRobot,
                                                                  scaledVictims, scaledGate);
        vector<Point *> shortestPath = computeShortestPath(&cleanestPaths, toReach);

        /*
         DebugImage::clear();
         DebugImage::drawGraph(cleanestPaths);
         DebugImage::drawPath(shortestPath);
         DebugImage::showAndWait();*/



        prunePath(&shortestPath, toReach);

        auto rescaledShortestPath = scaler.rescaleBackToDouble(toPointVector(shortestPath));

/*
        DebugImage::drawPath(shortestPath, cv::Scalar());
        DebugImage::showAndWait();*/


        vector<DubinsCurve> curves = findBestDubinsCurves(rescaledShortestPath,
                                                          theta,
                                                          &detector); // TODO: add obstacles to also do collision checking (and alsocheck border)

        vector<Pose> allPoses;
        for (auto curve:curves) {
            vector<Pose> poses = dubinsCurveToPoseVector(curve);
            allPoses.insert(allPoses.end(), poses.begin(), poses.end());
        }

        path.setPoints(allPoses);


        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        cout << "planning took " << elapsed_time_ms << "ms" << endl;

        DebugImage::clear();
        DebugImage::drawGraph(cleanestPaths);
        DebugImage::drawPoses(allPoses);
        //DebugImage::drawPath(shortestPath, cv::Scalar());
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


}