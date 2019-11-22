#include "dubins.h"

using namespace std;

DubinsCurve dubinsShortestPath(RobotPosition start, RobotPosition end, double kMax) {
    ScaledRobotPosition scale = scaleToStandard(start, end, kMax);
    cout << "t0=" << scale.thetaStart << " tf=" << scale.thetaEnd << " kmax=" << scale.kMax << " lambda=" << scale.lambda << endl;


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
    int pidx = 0, minPidx; // TODO: Rename pidx to i (for the for loop)
    for (auto solver : solvers) {
        auto res = solver->solve(scale);
        cout << "Length: " << res.length() << endl;
        if (res.ok && res.length() < minLength) {
            minLength = res.length();
            minCurve = res;
            minPidx = pidx;
        }
        pidx++;
    }
    cout << "Min pidx=" << minPidx << endl;

    // Back from standard
    cout << "s1=" << minCurve.sc_s1 << " s2=" << minCurve.sc_s2 << " s3=" << minCurve.sc_s3 << endl;
    DubinsResult scaled = minCurve.scaleFromStandard(scale.lambda);
    cout << "scaled s1=" << scaled.sc_s1 << " s2=" << scaled.sc_s2 << " s3=" << scaled.sc_s3 << endl;

    cout << "Minum scaled length: " << endl;
    cout << scaled.length() << endl;

    auto kSigns = solvers[minPidx]->getKSigns(kMax);
    return dubinsCurve(start.x, start.y, start.theta, scaled.sc_s1, scaled.sc_s2, scaled.sc_s3, kSigns.k0, kSigns.k1, kSigns.k2);
}

