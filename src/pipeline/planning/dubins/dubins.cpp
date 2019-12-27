#include "dubins.h"

using namespace std;

DubinsCurve dubinsShortestPath(RobotPosition start, RobotPosition end, double kMax) {
    ScaledRobotPosition scale = scaleToStandard(start, end, kMax);


    ManeuverSolver * solvers[] = {
            new LSL(),
            new RSR(),
            new LSR(),
            new RSL(),
            new RLR(),
            new LRL()
    };
    double minLength = INFINITY;
    DubinsResult minCurve;
    int minPidx; // TODO: Rename pidx to i (for the for loop)
    for (int i = 0; i < sizeof(solvers) / sizeof(ManeuverSolver*); i++) {
        auto solver = solvers[i];
        auto res = solver->solve(scale);
        if (res.ok && res.length() < minLength) {
            minLength = res.length();
            minCurve = res;
            minPidx = i;
        }
    }

    // Back from standard
    DubinsResult scaled = minCurve.scaleFromStandard(scale.lambda);


    auto kSigns = solvers[minPidx]->getKSigns(kMax);
    return dubinsCurve(start.x, start.y, start.theta, scaled.sc_s1, scaled.sc_s2, scaled.sc_s3, kSigns.k0, kSigns.k1, kSigns.k2);
}

