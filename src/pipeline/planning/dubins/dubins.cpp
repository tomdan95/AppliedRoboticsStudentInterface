#include <utils.hpp>
#include <algorithm>
#include "dubins.h"

using namespace std;

vector<DubinsCurve> dubinsShortestPath(RobotPosition start, RobotPosition end, double kMax) {
    StandardDubinsProblem scaledDubinsProblem = scaleToStandard(start, end, kMax);
    static ManeuverSolver *solvers[] = {
            new LSL(),
            new RSR(),
            new LSR(),
            new RSL(),
            new RLR(),
            new LRL()
    };

    vector<DubinsCurve> dubinsCurves;

    for (auto *solver:solvers) {
        auto res = solver->solve(scaledDubinsProblem);
        if (res.ok) {
            DubinsResult scaledRes = res.scaleFromStandard(scaledDubinsProblem.lambda);
            auto kSigns = solver->getKSigns(kMax);
            auto curve = dubinsCurve(start.x, start.y, start.theta, scaledRes.sc_s1, scaledRes.sc_s2, scaledRes.sc_s3,
                                     kSigns.k0, kSigns.k1, kSigns.k2);
            dubinsCurves.push_back(curve);
        }
    }

    sort(dubinsCurves.begin(), dubinsCurves.end(), [](DubinsCurve &a, DubinsCurve &b) {
        return a.length() < b.length();
    });
    return dubinsCurves;
}


vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve) {
    vector<Pose> poses;
    dubinsArcToPoseVector(curve.a1, poses);
    dubinsArcToPoseVector(curve.a2, poses);
    dubinsArcToPoseVector(curve.a3, poses);
    return poses;
}

void dubinsArcToPoseVector(DubinsArc arc, vector<Pose> &dst) {
    //const int numPoints = 100.0 * arc.length;
    const int numPoints = 30.0 * arc.length;
    const double delta = arc.length / (double)numPoints;
    for (int i = 0; i < numPoints; i++) {
        DubinsArc temp;
        double s = delta * ((double) i);

        circleLine(s, arc.x0, arc.y0, arc.th0, arc.k, &temp);
        // TODO: temp.k doesn't get updated. Do we need it in this data structure??
        dst.emplace_back(delta, temp.xf, temp.yf, temp.thf, arc.k);
    }
}
