#include <iostream>
#include <cmath>
#include "scale.h"
#include "temp.h"
#include "primitives.h"
#include "curve.h"

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
    DubinsResult * minCurve = NULL;
    int pidx = 0, minPidx; // TODO: Rename pidx to i (for the for loop)
    for (auto solver : solvers) {
        auto res = solver->solve(scale);
        cout << "Length: " << res.length() << endl;
        if (res.ok && res.length() < minLength) {
            minLength = res.length();
            minCurve = &res;
            minPidx = pidx;
        }
        pidx++;
    }


    // Back from standard
    DubinsResult scaled = minCurve->scaleFromStandard(scale.lambda);

    cout << "Minum scaled length: " << endl;
    cout << scaled.length() << endl;

    auto kSigns = solvers[minPidx]->getKSigns(kMax);
    return dubinsCurve(start.x, start.y, start.theta, scaled.sc_s1, scaled.sc_s2, scaled.sc_s3, kSigns.k0, kSigns.k1, kSigns.k2);
}



int main() {
    RobotPosition start(0, 0, (-2.0 / 3.0 * M_PI));
    RobotPosition end(4, 0, (M_PI / 3.0));
    double kMax = 3;

    auto res = dubinsShortestPath(start, end, kMax);

    cout << "Res: " << endl;
    cout << "Arc 1 xf=" << res.a1.xf << " yf=" << res.a1.yf << " thf=" << res.a1.thf << endl;
    cout << "Arc 2 xf=" << res.a2.xf << " yf=" << res.a2.yf << " thf=" << res.a2.thf << endl;
    cout << "Arc 3 xf=" << res.a3.xf << " yf=" << res.a3.yf << " thf=" << res.a3.thf << endl;
    cout << "Length=" << res.L << endl;
    return 0;
}


