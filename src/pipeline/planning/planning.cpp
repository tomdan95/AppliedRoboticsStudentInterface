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
#include "Mission1.h"

using namespace std;

namespace student {





    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {

        auto t_start = std::chrono::high_resolution_clock::now();
/*
        DebugImage::clear();
        DebugImage::drawPolygons(obstacleList, 800, cv::Scalar(255, 0, 0));
*/
        vector<Polygon> inflatedObstacles = inflateObstacles(obstacleList, borders, 30); //TODO:put correct robot size
/*
        DebugImage::drawPolygons(inflatedObstacles, 800, cv::Scalar(255, 255, 0));
        DebugImage::showAndWait();
*/

        CollisionDetector detector(obstacleList);// TODO: Not inflated!!
        Graph cleanestPaths = findCleanestPaths(inflatedObstacles, &detector);// TODO: OBSTACLES NOT INFLATED!!!

        auto solver = new Mission1(&detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getSortedVictimPoints(victimList));
        //auto solver2 = new Mission2(&detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getSortedVictimPoints(victimList), {10, 20, 30, 40});
        auto curves = solver->solve();

        vector<Pose> allPoses;
        for (auto curve:curves) {
            vector<Pose> poses = dubinsCurveToPoseVector(curve);
            allPoses.insert(allPoses.end(), poses.begin(), poses.end());
        }
        path.setPoints(allPoses);

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        cout << "planning took " << elapsed_time_ms << "ms" << endl;


        DebugImage::clear();
        DebugImage::drawGraph(cleanestPaths);
        DebugImage::drawPoses(allPoses);
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