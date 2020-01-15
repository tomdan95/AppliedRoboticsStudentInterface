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

        auto t_start = std::chrono::high_resolution_clock::now();
        double robotSize = config.getRobotSize();
        boost::optional<vector<DubinsCurve>> solution;

        do {
            cout << "[PLANNING] Using robot size: " << robotSize << endl;

            auto inflatedObstacles = inflateObstacles(obstacleList, robotSize);
            auto defaultedBorders = deflateArenaBorders(borders, robotSize);
            auto inflatedObstaclesAndDeflatedBorders = resizeObstaclesAndBorders(obstacleList, borders, robotSize);

            // instantiate collision detector. The intatiation is slow, so we do it only once here, and then we share the instance
            ShadowCollisionDetector detector(defaultedBorders[0], inflatedObstacles, gate,
                                             getVictimPolygons(victimList));

            // execute Voronoi to get the cleanest paths graph
            Graph cleanestPaths = findCleanestPaths(inflatedObstaclesAndDeflatedBorders, &detector);

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
            MissionSolver *solver;
            if (config.getMission() == 1) {
                solver = new Mission1(&detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate),
                                      getVictimPoints(victimList), config.getPruneThreshold());
            } else {
                solver = new Mission2(&detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate),
                                      getVictimPoints(victimList), config.getPruneThreshold(), config.getVictimBonus());
            }

            // execute the planning (sort victims + dijkstra + best_theta with collision detection)
            solution = solver->solve();
            if (!solution) {
                cout << "[PLANNING] Planning with robot size = " << robotSize << " failed. Trying again" << endl
                     << endl;
                robotSize -= 0.01;
            }
        } while (!solution && robotSize > 0);

        if (!solution) {
            cout << "[PLANNING] Planning failed. Giving up..." << endl;
            return false;
        }

        // print how much time the planning took
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        cout << "[PLANNING] Planning took " << elapsed_time_ms << "ms" << endl;

        auto poses = discretizeListOfDubinsCurves(*solution);
        path.setPoints(poses);

        // Draw the poses to the debug image
        DebugImage::drawPoses(poses);
        DebugImage::showAndWait();

        // planning was succesfull
        return true;
    }

    vector<Pose> discretizeListOfDubinsCurves(const vector<DubinsCurve> &curves) {
        vector<Pose> allPoses;
        for (const auto &curve:curves) {
            vector<Pose> poses = dubinsCurveToPoseVector(curve);
            allPoses.insert(allPoses.end(), poses.begin(), poses.end());
        }
        fixS(allPoses);
        return allPoses;
    }

    void fixS(vector<Pose> &poses) {
        double s = 0;
        for (auto &pose:poses) {
            s += pose.s;
            pose.s = s;
        }
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