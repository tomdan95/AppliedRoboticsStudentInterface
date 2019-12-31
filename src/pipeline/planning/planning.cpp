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
#include "best_theta/BestThetaFinder.h"
#include "inflate.h"
#include "Mission1.h"
#include "Mission2.h"

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


        auto solver = new Mission1(&detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getVictimPoints(victimList));
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
        DebugImage::drawPolygons(inflatedObstacles, 800, cv::Scalar(255, 255, 0));
        DebugImage::drawGraph(cleanestPaths);
        DebugImage::drawPoses(allPoses);
        DebugImage::showAndWait();

        return true;
    }


    vector<pair<int, Point>> getVictimPoints(const vector<pair<int, Polygon>> &victims) {
        vector<pair<int, Point>> victimPoints;
        for (const pair<int, Polygon> &victim:victims) {
            victimPoints.emplace_back(victim.first, getPolygonCenter(victim.second));
        }
        return victimPoints;
    }

}