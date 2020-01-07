/**
 * To execute this function, set 'planning' to false.
 */

#include <chrono>
#include "planning.h"
#include "../utils.h"
#include "voronoi/voronoi_cleanest_path.h"
#include "Graph.h"
#include "../DebugImage.h"
#include "inflate.h"
#include "Mission1.h"
#include "Mission2.h"
#include "collision_detection/ShadowCollisionDetector.h"
#include "../Config.h"

using namespace std;

namespace student {

    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {

        cout << configFolder << endl;

        Config config(configFolder + "/config.json");

        double robotSize = config.getRobotSize();

        auto inflatedObstacles = inflateObstacles(obstacleList, robotSize);
        auto defaultedBorders = deflateArenaBorders(borders, robotSize);
        auto inflatedObstaclesAndDeflatedBorders = resizeObstaclesAndBorders(obstacleList, borders, robotSize);

        auto t_start = std::chrono::high_resolution_clock::now();

        CollisionDetector* detector = new ShadowCollisionDetector(defaultedBorders[0], inflatedObstacles, gate);
        Graph cleanestPaths = findCleanestPaths(inflatedObstaclesAndDeflatedBorders, detector);

        DebugImage::clear();
        DebugImage::drawPoint(Point(x, y));
        DebugImage::drawPolygon(borders, 400);
        DebugImage::drawPolygons(defaultedBorders, 400);
        DebugImage::drawPolygons(obstacleList, 400, cv::Scalar(255, 100, 0));
        DebugImage::drawPolygons(inflatedObstacles, 400, cv::Scalar(255, 255, 0));
        DebugImage::drawGraph(cleanestPaths);
        DebugImage::showAndWait();


        MissionSolver* solver;
        if(config.getMission() == 1) {
            solver = new Mission1(detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getVictimPoints(victimList));
        } else {
            solver = new Mission2(detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getVictimPoints(victimList), config.getVictimBonus());
        }
        auto curves = solver->solve();

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        cout << "planning took " << elapsed_time_ms << "ms" << endl;


        if(!curves) {
            cout << "planning failed" << endl;


            return false;
        }

        vector<Pose> allPoses;
        for (auto curve:*curves) {
            vector<Pose> poses = dubinsCurveToPoseVector(curve);
            allPoses.insert(allPoses.end(), poses.begin(), poses.end());
        }
        path.setPoints(allPoses);

        DebugImage::clear();
        DebugImage::drawPolygon(borders, 400);
        DebugImage::drawPolygons(defaultedBorders, 400);
        DebugImage::drawPolygons(inflatedObstacles, 400, cv::Scalar(255, 255, 0));
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