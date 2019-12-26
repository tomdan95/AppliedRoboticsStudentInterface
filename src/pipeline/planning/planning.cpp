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

#define INT_ROUND 1000.0

using namespace std;

namespace student {

    void drawCleanestPath(Graph graph) {
        cv::Mat image(1000, 1280, CV_8UC3, cv::Scalar(0, 0, 255));
        for(const auto edge : graph.edges) {
            cv::Point start(edge.first.x, edge.first.y);
            cv::Point end(edge.second.x, edge.second.y);
            cv::line(image, start, end, cv::Scalar(255, 0, 0));
        }
        showImageAndWaitKeyPress(image);
    }

    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {
        vector<Polygon> inflatedObstacles = inflateObstacles(obstacleList, borders);
        Graph cleanestPaths = findCleanestPaths(inflatedObstacles, obstacleList);// TODO: OBSTACLES NOT INFLATED!!!

        drawCleanestPath(cleanestPaths);

        // TODO: Connect the Voronoi path from the robot to the first victim, then from the first victmin to the second
        // TODO: and so on, until we connect the last victim to the gate
        // TODO: Every time we compute the shortes path fom one point to the other, we consider only the Voronoi vertexes
        // TODO: and then we compute dubins (making sure that the curve doesn't intersect an obstacle)

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