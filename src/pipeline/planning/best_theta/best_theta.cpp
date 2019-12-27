#include "best_theta.h"
#include "../dubins/curve.h"
#include "../dubins/dubins.h"


using namespace std;


namespace student {

    DubinsCurve invokeDubins(double startTheta, double endTheta, Point startPoint, Point endPoint) {
        RobotPosition start(startPoint.x, startPoint.y, startTheta);
        RobotPosition end(endPoint.x, endPoint.y, endTheta);
        return dubinsShortestPath(start, end, 10);// TODO: Remove hardcoded k
    }

    vector<DubinsCurve> findBestTheta(vector<Point *> path, double robotTheta) {
        int steps = 64;
        double step = (2.0 * M_PI) / steps;
        vector<vector<pair<pair<DubinsCurve, int>, double>>> curvesTable;
        curvesTable.resize(path.size() - 1);

        for (int beforeEnd = 0; beforeEnd < steps; beforeEnd++) {
            for (int end = 0; end < steps; end++) {
                DubinsCurve curve = invokeDubins(beforeEnd * step, end * step, *path[path.size() - 2],
                                                 *path[path.size() - 1]);
                curvesTable[path.size() - 2].emplace_back(make_pair(curve, -1), curve.L);
            }
        }


        for (int pathPoint = path.size() - 3; pathPoint >= 1; pathPoint--) {
            for (int beforeI = 0; beforeI < steps; beforeI++) {
                for (int i = 0; i < steps; i++) {
                    DubinsCurve curve = invokeDubins(beforeI * step, i * step, *path[pathPoint], *path[pathPoint + 1]);
                    curvesTable[pathPoint].emplace_back(make_pair(curve, i),
                                                        curve.L + curvesTable[pathPoint + 1][i].second);
                }
            }
        }


        for (int i = 0; i < steps; i++) {
            DubinsCurve curve = invokeDubins(robotTheta, i * step, *path[0], *path[1]);
            curvesTable[0].emplace_back(make_pair(curve, i), curve.L + curvesTable[1][i].second);
        }


        vector<DubinsCurve> curves;
        double minLength = 100000;
        int minIndex = -1;
        for (int i = 0; i < steps; i++) {
            if (curvesTable[0][i].second < minLength) {
                minLength = curvesTable[0][i].second;
                minIndex = i;
            }
        }
        curves.push_back(curvesTable[0][minIndex].first.first);


        int pre = curvesTable[0][minIndex].first.second;
        int pathPoint = 1;
        while (pre != -1) {
            curves.push_back(curvesTable[pathPoint][pre].first.first);
            pre = curvesTable[pathPoint][pre].first.second;
            pathPoint++;
        }

        for (auto curve : curves) {
            cout << curve.L << endl;
        }


        cout << "generated " << curves.size() << "curves" << endl;
        return curves;
    }
}