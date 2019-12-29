#include <utils.hpp>
#include "dubins.h"

using namespace std;

DubinsCurve dubinsShortestPath(RobotPosition start, RobotPosition end, double kMax) {
    StandardDubinsProblem scaledDubinsProblem = scaleToStandard(start, end, kMax);
    static ManeuverSolver * solvers[] = {
            new LSL(),
            new RSR(),
            new LSR(),
            new RSL(),
            new RLR(),
            new LRL()
    };
    double minLength = INFINITY;
    DubinsResult minCurve;
    int minPidx = -1; // TODO: Rename pidx to i (for the for loop)
    for (int i = 0; i < sizeof(solvers) / sizeof(ManeuverSolver*); i++) {
        auto solver = solvers[i];
        auto res = solver->solve(scaledDubinsProblem);
        if (res.ok && res.length() < minLength) {
            minLength = res.length();
            minCurve = res;
            minPidx = i;
        }
    }

    if (minPidx == -1) {
        DubinsCurve curve;
        // TODO: Return pointers, so we can return NULL
        return curve;
    }

    // Back from standard
    DubinsResult scaled = minCurve.scaleFromStandard(scaledDubinsProblem.lambda);


    auto kSigns = solvers[minPidx]->getKSigns(kMax);
    return dubinsCurve(start.x, start.y, start.theta, scaled.sc_s1, scaled.sc_s2, scaled.sc_s3, kSigns.k0, kSigns.k1, kSigns.k2);
}


vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve) {
    vector<Pose> poses;
    dubinsArcToPoseVector(curve.a1, poses);
    dubinsArcToPoseVector(curve.a2, poses);
    dubinsArcToPoseVector(curve.a3, poses);
    return poses;
}

void dubinsArcToPoseVector(DubinsArc arc, vector<Pose> &dst) {
    const int numPoints = 50.0 * arc.length;
    for (int i = 0; i < numPoints; i++) {
        DubinsArc temp;
        double s = arc.length / numPoints * ((float) i);
        circleLine(s, arc.x0, arc.y0, arc.th0, arc.k, &temp);
        // TODO: temp.k doesn't get updated. Do we need it in this data structure??
        dst.emplace_back(1, temp.xf, temp.yf, temp.thf, arc.k);
    }
}


