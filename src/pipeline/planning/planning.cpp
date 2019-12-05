/**
 * To execute this function, set 'planning' to false.
 */

#include "planning.h"
#include <chrono>
#include "clipper/clipper.hpp"

#define INT_ROUND 1000

using namespace std;
namespace student {

    vector<Polygon> inflateObstacles(const vector<Polygon> &obstacles) {

        vector<Polygon> inflatedObstacles;
        for (const Polygon &obstacle : obstacles) {
            ClipperLib::Path clipperObstacle;
            ClipperLib::Paths clipperInflated;
            for (const auto &point : obstacle) {
                clipperObstacle << ClipperLib::IntPoint(point.x * INT_ROUND, point.y * INT_ROUND);
            }

            ClipperLib::ClipperOffset co;
            if (obstacle.size() == 3) {
                co.AddPath(clipperObstacle, ClipperLib::jtSquare, ClipperLib::etClosedLine);
            } else {
                co.AddPath(clipperObstacle, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
            }

            co.Execute(clipperInflated, 10); // TODO: Chane the offset value

            for (const auto &inflatedPath : clipperInflated) {
                Polygon inflatedObstacle;
                for (const auto&point : inflatedPath) {
                    int x = point.X / INT_ROUND;
                    int y = point.Y / INT_ROUND;
                    inflatedObstacle.emplace_back(x, y);
                }
                inflatedObstacles.push_back(inflatedObstacle);
            }
        }
        return inflatedObstacles;
    }

    bool planPath(const Polygon &borders, const vector<Polygon> &obstacleList,
                  const vector<pair<int, Polygon>> &victimList,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const string &configFolder) {

        cout << "[PLANNING] begin planPath" << endl;
        auto startTime = chrono::high_resolution_clock::now();


        vector<Polygon> inflatedObstacles = inflateObstacles(obstacleList);



        /*
        vector<Pose> poseVector;
        RobotPosition start(x, y, theta);
        RobotPosition gateEnd(gate[0].x, gate[0].y, (M_PI / 3.0));

        for (int i = 0; i <= 5; i++)
            for (const pair<int, Polygon> &victim:victimList) {
                if(victim.first == i) {
                    RobotPosition poseForVictim(victim.second[0].x, victim.second[0].y, theta);
                    auto res = dubinsShortestPath(start, poseForVictim, 5);
                    dubinsCurveToPoseVector(res, poseVector);
                    start = poseForVictim;
                }
            }

        auto res = dubinsShortestPath(start, gateEnd, 5);
        dubinsCurveToPoseVector(res, poseVector);


        path.setPoints(poseVector);
*/
        auto endTime = chrono::high_resolution_clock::now();
        auto timeTook = endTime - startTime;
        auto timeTookMs = chrono::duration_cast<chrono::milliseconds>(timeTook).count();
        cout << "[PLANNING] end planPath. Total time: " << timeTookMs << endl;
        return true;
    }


    vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve, vector<Pose> &vector) {
        dubinsArcToPoseVector(curve.a1, vector);
        dubinsArcToPoseVector(curve.a2, vector);
        dubinsArcToPoseVector(curve.a3, vector);
        return vector;
    }


    void dubinsArcToPoseVector(DubinsArc arc, vector<Pose> &vector) {
        /**
         * npts = 100;
  pts = zeros(npts+1, 2);
  for j = 0:npts
    s = arc.L/npts * j;
    [x, y] = circline(s, arc.x0, arc.y0, arc.th0, arc.k);
    pts(j+1, 1:2) = [x, y];
  end
         */
        /*

       DubinsArc temp;
       Pose pose;
       circleLine(0, arc.x0, arc.y0, arc.th0, arc.k, &temp);
       vector.push_back(pose);
       */

        const int numPoints = 50;
        for (int i = 0; i < numPoints; i++) {
            DubinsArc temp;
            Pose pose;
            double s = arc.L / numPoints * i;
            circleLine(s, arc.x0, arc.y0, arc.th0, arc.k, &temp);
            pose.x = temp.x0;
            pose.y = temp.y0;
            vector.emplace_back(1, temp.xf, temp.yf, temp.thf, temp.k);
        }
    }

}