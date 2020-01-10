#include "BestThetaFinder.h"
#include "../dubins/dubins.h"
#include "../collision_detection/CollisionDetector.h"

#define STEPS 32
#define STEP  ((2.0 * M_PI) / STEPS)

using namespace std;
using namespace boost;

namespace student {


    optional<vector<DubinsCurve>> BestThetaFinder::findBestDubinsCurves(const vector<Point *> &path) {
        initializeTable(path);
        fillTableFromLastToFirstPathPoint(path);
        fillTableWithFirstPathPoint(path);
        return getBestCurvesFromTable();
    }

    void BestThetaFinder::initializeTable(const vector<Point *> &path) {
        curvesTable.clear();
        curvesTable.resize(path.size() - 1);
    }

    inline void BestThetaFinder::fillTableFromLastToFirstPathPoint(const vector<Point *> &path) {
        for (int start = path.size() - 2; start >= 1; start--) {
            computeTableRow(start, path);
        }
    }

    inline void BestThetaFinder::computeTableRow(int start, const vector<Point *> &path) {
        for (int startTheta = 0; startTheta < STEPS; startTheta++) {
            computeTableCell(start, startTheta, path);
        }
    }

    inline void BestThetaFinder::computeTableCell(int start, int startTheta, const vector<Point *> &path) {
        int end = start + 1;
        auto startPoint = *path[start];
        auto endPoint = *path[end];
        bool isLast = start == path.size() - 2;

        optional<vector<optional<SubPathCurve>>> after;
        if (!isLast) {
            after.emplace(curvesTable[end]);
        }

        auto bestCurveForStartingTheta = findBestCurveForStartingTheta(RobotPosition(startPoint, startTheta * STEP),
                                                                       endPoint, after);
        curvesTable[start].push_back(bestCurveForStartingTheta);
    }

    inline optional<SubPathCurve>
    BestThetaFinder::findBestCurveForStartingTheta(const RobotPosition &start, const Point &endPoint,
                                                   optional<vector<optional<SubPathCurve>>> after) {
        optional<SubPathCurve> bestCurve;
        for (int endTheta = 0; endTheta < STEPS; endTheta++) {
            /**
             * If the subpath has a successor (it's not the last), we'll need to connect it to the successor. But if the
             * successor doesn't exists there is no point in computing the curve.
             */
            if (after && !(*after)[endTheta]) {
                continue;
            }
            auto curve = findShortestNotCollidingCurve(start, RobotPosition(endPoint, endTheta * STEP));
            if (!curve) {
                continue;
            }

            double length = curve->length();
            if (after) {
                length += (*after)[endTheta]->length;
            }

            if (!bestCurve || length < bestCurve->length) {
                SubPathCurve subPath(*curve, endTheta, length);
                bestCurve.emplace(subPath);
            }
        }
        return bestCurve;
    }

    inline void BestThetaFinder::fillTableWithFirstPathPoint(const vector<Point *> &path) {
        for (int endTheta = 0; endTheta < STEPS; endTheta++) {
            if (!curvesTable[1][endTheta]) {
                curvesTable[0].push_back(none);
                continue;
            }
            auto curve = findShortestNotCollidingCurve(
                    RobotPosition(*path[0], startingTheta),
                    RobotPosition(*path[1], endTheta * STEP)
            );
            if (!curve) {
                curvesTable[0].push_back(none);
            } else {
                SubPathCurve subPathCurve(*curve, endTheta, curve->length() + curvesTable[1][endTheta]->length);
                curvesTable[0].emplace_back(subPathCurve);
            }
        }
    }

    optional<DubinsCurve>
    BestThetaFinder::findShortestNotCollidingCurve(const RobotPosition &start, const RobotPosition &end) const {
        auto curves = dubinsShortestPath(start, end, maxK);
        for (const auto &curve:curves) {
            //if (!collisionDetector->doesCurveCollide(curve)) {
                return curve;
            //}
        }
        return none;
    }

    optional<vector<DubinsCurve>> BestThetaFinder::getBestCurvesFromTable() const {
        auto bestTheta = findBestFirstPointArrivingTheta();
        if (!bestTheta){
            return none;
        }
        return buildBestCurvesVector(*bestTheta);
    }

    optional<int> BestThetaFinder::findBestFirstPointArrivingTheta () const {
        double minLength = INFINITY;
        int bestTheta = -1;
        for (int i = 0; i < STEPS; i++) {
            if (curvesTable[0][i] && curvesTable[0][i]->length < minLength) {
                minLength = curvesTable[0][i]->length;
                bestTheta = i;
            }
        }
        if (minLength == INFINITY) {
            return none;
        }
        return bestTheta;
    }

    vector<DubinsCurve> BestThetaFinder::buildBestCurvesVector(int bestTheta) const {
        vector<DubinsCurve> curves;
        for (int startPathPoint = 0; startPathPoint < curvesTable.size(); startPathPoint++) {
            auto subPathCurve = curvesTable[startPathPoint][bestTheta];
            if(!subPathCurve) {
                cout << "problem" << endl;
                // TODO: Handle (this shouldn't happen)
            }
            curves.push_back(subPathCurve->curve);
            bestTheta = subPathCurve->arrivingTheta;
        }
        return curves;
    }




}