#include "best_theta.h"
#include "../dubins/curve.h"
#include "../dubins/dubins.h"
#include "../collision_detection/CollisionDetector.h"

#define STEPS 32
#define STEP  ((2.0 * M_PI) / STEPS)
#define MAX_K 10

using namespace std;


namespace student {

    DubinsCurve invokeDubins(double startTheta, double endTheta, Point startPoint, Point endPoint) {
        RobotPosition start(startPoint.x, startPoint.y, startTheta);
        RobotPosition end(endPoint.x, endPoint.y, endTheta);
        return dubinsShortestPath(start, end, MAX_K);
    }

    /**
     * Computes the best Dubins curves using multipoint Dubins interpolation.
     *
     * Description of the algorithm:
     *
     * Our problem is that we want to make the robot follow a path (list of points). Th robot moves on a 2D surface,
     * so it has an x and y coordinate. The robot also has a starting orientation (theta). We don't care about which
     * theta the robot will have when it reaches a point of the path, but we want to compute the shortest path.
     * It is important to notice that the shortest path may consist of non-local optima curve, so we cannot use a greedy
     * approach. This algorithm is indeed based on Dynamic programming.
     *
     * Steps of the algorithm:
     * - we choose a finite number of angles that we want to try. Currently this number is defined ad a macro. We will
     *   use this number to evenly divide the range of possible orientations for which we can leave/reach a point.
     * - we compute all the possibles Dubins curves to reach last point (j) from the previous point (i) with all the
     *   possible starting and arriving orientations. We store in a table the shortest curve for each possible starting
     *   orientation.
     *   We know know the best curve to reach point j from point i given any possible starting orientation from i
     * - we now compute all the Dubins curves from point i - 1 to i. This time, instead of storing the shortest curve
     *   into the table, for each starting orientation we store the curve that, when summed with the previous curves is
     *   shortest (subpath from i to j). We repeat this until we reach the starting point (each time the subpath is
     *   longer).
     * - we know chose the best arriving theta from the first point to the second point
     *
     * Once we filled the table, we can use it to create the shortest list of curves.
     *
     * @param path Path that the robot has to follow
     * @param robotTheta Starting theta of the robot
     * @return list of curves that the robot has to follow
     */
    vector<DubinsCurve>
    findBestDubinsCurves(vector<Point *> path, double robotTheta, CollisionDetector *collisionDetector) {
        /**
         * Table that contains the curves.
         * It is a vector that contains a vector for each edge of the path.
         * Each vector associated to a path contains a vector of best curves. These vectors have the same size as the
         * number of orientations that we're going to try. For each curve we store the curve itself, the arriving theta
         * and the length of the subpath (from that point to the end).
         */
        vector<vector<pair<pair<DubinsCurve, int>, double>>> curvesTable;
        curvesTable.resize(path.size() - 1);

        // fill the table, starting from the last point
        for (int pathPoint = path.size() - 2; pathPoint >= 1; pathPoint--) {
            for (int thetaI = 0; thetaI < STEPS; thetaI++) { // we try with every starting orientation
                DubinsCurve shortestCurveForThetaI;
                double lengthOfShortestCurve = INFINITY;
                int thetaJForShortestCurve = -1;
                for (int thetaJ = 0; thetaJ < STEPS; thetaJ++) { // we try with every arriving orientation
                    DubinsCurve curve = invokeDubins(thetaI * STEP, thetaJ * STEP, *path[pathPoint], *path[pathPoint + 1]);
                    double length = curve.L;
                    if (pathPoint < path.size() - 2) {
                        // if the curve doesn't reach the last point, we add the length of the subpath corresponding
                        // to the arriving theta
                        length += curvesTable[pathPoint + 1][thetaJ].second;
                    }
                    if(length < lengthOfShortestCurve && !collisionDetector->doesCurveCollide(curve)) {
                        lengthOfShortestCurve = length;
                        shortestCurveForThetaI = curve;
                        thetaJForShortestCurve = thetaJ;
                    }
                }
                if(thetaJForShortestCurve == -1){
                    // TODO: Handle this case
                    cout << "not found!!!!!!!!" << endl;
                }
                if (pathPoint == path.size() - 2) {
                    // if the curve reaches the last point, we're n ot interested in the arriving theta.
                    // we store -1 so later we know when we computed the list of curves
                    thetaJForShortestCurve = -1;
                }
                curvesTable[pathPoint].emplace_back(make_pair(shortestCurveForThetaI, thetaJForShortestCurve), lengthOfShortestCurve);
            }
        }

        // we already have a starting theta. Compute the curves from the starting point to the second point, using
        // all possible arriving orientations
        for (int i = 0; i < STEPS; i++) {
            DubinsCurve curve = invokeDubins(robotTheta, i * STEP, *path[0], *path[1]);
            if (collisionDetector->doesCurveCollide(curve)) {
                // TODO: Handle this case
                cout << "not found!!!!!!!!" << endl;
                curvesTable[0].emplace_back(make_pair(curve, i), 1000);
                // TODO: Handle case were all the first curves collide
            } else {
                curvesTable[0].emplace_back(make_pair(curve, i), curve.L + curvesTable[1][i].second);
            }
        }

        // find the first curve of the shortest list of curves
        double minLength = INFINITY;
        int bestTheta = -1;
        for (int i = 0; i < STEPS; i++) {
            if (curvesTable[0][i].second < minLength) {
                minLength = curvesTable[0][i].second;
                bestTheta = i;
            }
        }

        // build the list of curves
        vector<DubinsCurve> curves;
        int pathPoint = 0;
        while (bestTheta != -1) {
            curves.push_back(curvesTable[pathPoint][bestTheta].first.first);
            bestTheta = curvesTable[pathPoint][bestTheta].first.second;
            pathPoint++;
        }
        return curves;
    }
}