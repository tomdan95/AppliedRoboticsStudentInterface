#ifndef STUDENT_PROJECT_BEST_THETA_H
#define STUDENT_PROJECT_BEST_THETA_H

#include <utils.hpp>
#include <boost/optional.hpp>
#include "../collision_detection/CollisionDetector.h"
#include "../dubins/models.h"

using namespace std;
using namespace boost;

namespace student {

    class SubPathCurve {
    public:
        const DubinsCurve curve;
        const int arrivingTheta;
        const double length;

        SubPathCurve(
                const DubinsCurve &curve,
                int arrivingTheta,
                double length
        ) : curve(curve),
            arrivingTheta(arrivingTheta),
            length(length) {}
    };


    class BestThetaFinder {
    private:
        const int maxK;
        const double startingTheta;
        const int STEPS;
        const double STEP;
        const CollisionDetector *collisionDetector;

        /**
         * Table that contains the curves.
         * It is a vector that contains a vector for each edge of the path.
         * Each vector associated to a path contains a vector of best curves. These vectors have the same size as the
         * number of orientations that we're going to try. For each curve we store the curve itself, the arriving theta
         * and the length of the subpath (from that point to the end).
         */
        vector<vector<optional<SubPathCurve>>> curvesTable;

        void initializeTable(const vector<Point *> &path);

        void fillTableFromLastToFirstPathPoint(const vector<Point *> &path);

        void fillTableWithFirstPathPoint(const vector<Point *> &path);

        void computeTableRow(int start, const vector<Point *> &path);

        void computeTableCell(int start, int startTheta, const vector<Point *> &path);

        optional<SubPathCurve>
        findBestCurveForStartingTheta(const RobotPosition &start, const Point &endPoint,
                                      optional<vector<optional<SubPathCurve>>> after);

        optional<vector<DubinsCurve>> getBestCurvesFromTable() const;


        optional<DubinsCurve>
        findShortestNotCollidingCurve(const RobotPosition &start, const RobotPosition &end) const;

        optional<int> findBestFirstPointArrivingTheta() const;

        vector<DubinsCurve> buildBestCurvesVector(int bestTheta) const;

    public:
        BestThetaFinder(const int steps, const int maxK, const double robotTheta,
                        const CollisionDetector *collisionDetector) :
                STEPS(steps),STEP((2.0 * M_PI) / steps), maxK(maxK), startingTheta(robotTheta), collisionDetector(collisionDetector){}

        /**
         * Computes the best Dubins curves using multipoint Dubins interpolation.
         *
         * Description of the algorithm:
         *
         * Our problem is that we want to make the robot follow a path (list of points). The robot moves on a 2D surface,
         * so it has an x and y coordinate. The robot also has a starting orientation (theta). We don't care about which
         * theta the robot will have when it reaches a point of the path, but we want to compute the shortest path.
         * It is important to notice that the shortest path may consist of non-local optima curve, so we cannot use a greedy
         * approach. This algorithm is indeed based on Dynamic programming.
         *
         * Steps of the algorithm:
         * - we choose a finite number of angles that we want to try. We will use this number to evenly divide the range of
         *   possible orientations for which we can leave/reach a point.
         * - we compute all the possible Dubins curves to reach last point from the pre-last point with all the possible
         *   starting and arriving orientations. We store in a table the shortest curve for each possible starting
         *   orientation.
         *   We now know the best curve to reach last point from pre-last point, given any possible starting orientation
         *   from pre-last
         * - we now compute all the Dubins curves from point i - 1 to i. This time, instead of storing the shortest curve
         *   into the table, for each starting orientation we store the curve that, when summed with the previous curves is
         *   shortest (subpath from i to j). We repeat this until we reach the starting point (each time the subpath is
         *   longer).
         * - we know chose the best arriving theta from the first point to the second point
         *
         * Once we filled the table, we can use it to create the shortest list of curves.
         *
         * @param path Path that the robot has to follow
         * @return list of curves that the robot has to follow
        */
        optional<vector<DubinsCurve>> findBestDubinsCurves(const vector<Point *> &path);
    };

}

#endif
