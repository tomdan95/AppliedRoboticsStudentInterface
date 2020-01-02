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
#include "collision_detection/ShadowCollisionDetector.h"

using namespace std;

namespace student {





    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {

        int robotSize = 30;
        auto inflatedObstacles = inflateObstacles(obstacleList, robotSize);
        auto defaultedBorders = deflateArenaBorders(borders, robotSize);
        auto inflatedObstaclesAndDeflatedBorders = resizeObstaclesAndBorders(obstacleList, borders, robotSize);

        auto t_start = std::chrono::high_resolution_clock::now();

        CollisionDetector* detector = new ShadowCollisionDetector(defaultedBorders[0], inflatedObstacles);
        Graph cleanestPaths = findCleanestPaths(inflatedObstaclesAndDeflatedBorders, detector);


        auto solver = new Mission1(detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getVictimPoints(victimList));
        //auto solver2 = new Mission2(&detector, &cleanestPaths, RobotPosition(x, y, theta), getPolygonCenter(gate), getSortedVictimPoints(victimList), {10, 20, 30, 40});
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