/**
 * To execute this function, set 'planning' to false.
 */

#include "planning.h"
#include <chrono>
#include "clipper/clipper.hpp"
#include "../utils.h"

#include "../planning/voronoi_helper.h"

#define INT_ROUND 1000.0

using namespace std;
namespace student {


    void testVoronoiPlanning(vector<Polygon> &vector) {
        cv::Mat image(1000, 1280, CV_8UC3, cv::Scalar(0, 0, 255));
        testComputeVoronoi(image, vector);
    }

    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {
        cout << "[PLANNING] begin planPath" << endl;
        auto startTime = chrono::high_resolution_clock::now();

        vector<Polygon> inflatedObstacles = inflateObstacles(obstacleList);
        // TODO: Now that the obstacles has been inflated, merge some of them (the ones which overlap)

        //vector<Polygon> copy = inflatedObstacles;


        vector<Polygon> copy = inflatedObstacles;
        testVoronoiPlanning(copy);


        vector<Point> pathPoints = getSortedVictimPoints(victimList);
        pathPoints.push_back(getPolygonCenter(gate));

        vector<Pose> poseVector;
        RobotPosition start(x, y, theta);
        for (const Point &point:pathPoints) {
            RobotPosition pointWithTheta(point.x, point.y, 0);
            auto res = dubinsShortestPath(start, pointWithTheta, 5);
            dubinsCurveToPoseVector(res, poseVector);
            start = pointWithTheta;
        }


        path.setPoints(poseVector);

        auto endTime = chrono::high_resolution_clock::now();
        auto timeTook = endTime - startTime;
        auto timeTookMs = chrono::duration_cast<chrono::milliseconds>(timeTook).count();
        cout << "[PLANNING] end planPath. Total time: " << timeTookMs << endl;
        return true;
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

    vector<Polygon> inflateObstacles(const vector<Polygon> &obstacles) {
        vector<Polygon> returnObstacles;
        ClipperLib::Clipper cl;
        ClipperLib::Paths MergedObstacles;
        for (const Polygon &obstacle : obstacles) {
            ClipperLib::Path clipperObstacle;
            ClipperLib::Paths clipperInflatedObstacle;
            for (const auto &point : obstacle) {
                clipperObstacle << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
            }
            clipperObstacle << ClipperLib::IntPoint(obstacle[0].x * INT_ROUND, obstacle[0].y * INT_ROUND);

            ClipperLib::ClipperOffset co;
            cout << "inflating, size " << obstacle.size() << endl;
            // TODO: Do we need to do something different for triangles?
            //if (obstacle.size() == 3) {
            //    co.AddPath(clipperObstacle, ClipperLib::jtSquare, ClipperLib::etClosedLine);
            //} else {
            co.AddPath(clipperObstacle, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
            //}

            co.Execute(clipperInflatedObstacle, 20); // TODO: Change the offset value according to the robot size
            cl.AddPaths(clipperInflatedObstacle, ClipperLib::ptSubject, true);
        }
        cl.Execute(ClipperLib::ctUnion, MergedObstacles, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        for (const auto &mergedPath : MergedObstacles) {
                Polygon mergedObstacle;
                for (const auto &point : mergedPath) {
                    mergedObstacle.emplace_back(point.X / INT_ROUND, point.Y / INT_ROUND);
                }
                //inflatedObstacle.erase()
                returnObstacles.push_back(mergedObstacle);
            }
        return returnObstacles;
    }


    vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve, vector<Pose> &vector) {
        dubinsArcToPoseVector(curve.a1, vector);
        dubinsArcToPoseVector(curve.a2, vector);
        dubinsArcToPoseVector(curve.a3, vector);
        return vector;
    }


    void dubinsArcToPoseVector(DubinsArc arc, vector<Pose> &vector) {
        const int numPoints = 5;
        for (int i = 0; i < numPoints; i++) {
            DubinsArc temp;
            double s = arc.L / numPoints * ((float) i);
            circleLine(s, arc.x0, arc.y0, arc.th0, arc.k, &temp);
            // TODO: temp.k doesn't get updated. Do we need it in this data structure??
            vector.emplace_back(1, temp.xf, temp.yf, temp.thf, arc.k);
        }
    }

}