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
    DubinsCurve * minCurve = NULL;
    for (auto solver : solvers) {
        auto res = solver->solve(scale);
        cout << res.length() << endl;
        if (res.ok && res.length() < minLength) {
            minLength = res.length();
            minCurve = &res;
        }
    }


    // Back from standard
    DubinsCurve scaled = minCurve->scaleFromStandard(scale.lambda);

    cout << "Scaled:" << endl;
    cout << scaled.length() << endl;

    //dubinsCurve(start, scaled, ???)
    return {};
}



int main() {
    RobotPosition start(0, 0, (-2.0 / 3 * M_PI));
    RobotPosition end(4, 0, (M_PI / 3.0));
    double kMax = 3;

    auto res = dubinsShortestPath(start, end, kMax);


    return 0;
}


