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

    vector<Polygon> getVictimPolygons(const vector<pair<int, Polygon>> &victims);

    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {

        Config config(configFolder);

        double robotSize = config.getRobotSize();
        cout << "using robot size: " << robotSize << endl;

        auto inflatedObstacles = inflateObstacles(obstacleList, robotSize);
        auto defaultedBorders = deflateArenaBorders(borders, robotSize);
        auto inflatedObstaclesAndDeflatedBorders = resizeObstaclesAndBorders(obstacleList, borders, robotSize);

        auto t_start = std::chrono::high_resolution_clock::now();

        // instantiate collision detector. The intatiation is slow, so we do it only once here, and then we share the instance
        CollisionDetector* detector = new ShadowCollisionDetector(defaultedBorders[0], inflatedObstacles, gate, getVictimPolygons(victimList));

        // execute Voronoi to get the cleanest paths graph
        Graph cleanestPaths = findCleanestPaths(inflatedObstaclesAndDeflatedBorders, detector);

        // draw map for debugging purposes
        DebugImage::clear();
        DebugImage::drawPoint(Point(x, y));
        DebugImage::drawPolygon(borders, 400);
        DebugImage::drawPolygons(defaultedBorders, 400);
        DebugImage::drawPolygons(obstacleList, 400, cv::Scalar(255, 100, 0));
        DebugImage::drawPolygons(inflatedObstacles, 400, cv::Scalar(255, 255, 0));
        DebugImage::drawGraph(cleanestPaths);
        DebugImage::showAndWait();


        // create the MissionSolver accordingly to the configuration file
        MissionSolver* solver;
        if(config.getMission() == 1) {
            solver = new Mission1(detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getVictimPoints(victimList));
        } else {
            solver = new Mission2(detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getVictimPoints(victimList), config.getVictimBonus());
        }

        // execute the planning (sort victims + dijkstra + best_theta with collision detection)
        auto curves = solver->solve();

        // print how much time the planning took
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        cout << "planning took " << elapsed_time_ms << "ms" << endl;

        // if the MissionSolver didn't gave us a list of DubisCurve, the planning failed
        if(!curves) {
            cout << "planning failed" << endl;
            return false;
        }

        // Discretize the DubinsCurves to a list of poses
        vector<Pose> allPoses;
        for (auto curve:*curves) {
            vector<Pose> poses = dubinsCurveToPoseVector(curve);
            allPoses.insert(allPoses.end(), poses.begin(), poses.end());
        }
        path.setPoints(allPoses);

        // Draw the poses to the debug image
        DebugImage::drawPoses(allPoses);
        DebugImage::showAndWait();

        // planning was succesfull
        return true;
    }


    vector<pair<int, Point>> getVictimPoints(const vector<pair<int, Polygon>> &victims) {
        vector<pair<int, Point>> victimPoints;
        for (const pair<int, Polygon> &victim:victims) {
            victimPoints.emplace_back(victim.first, getPolygonCenter(victim.second));
        }
        return victimPoints;
    }


    vector<Polygon> getVictimPolygons(const vector<pair<int, Polygon>> &victims) {
        vector<Polygon> polygons;
        for (const pair<int, Polygon> &victim:victims) {
            polygons.emplace_back(victim.second);
        }
        return polygons;
    }

}