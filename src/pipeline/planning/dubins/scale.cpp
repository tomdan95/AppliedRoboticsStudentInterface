#include "scale.h"

ScaledRobotPosition scaleToStandard(RobotPosition start, RobotPosition end, double kMax) {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double phi = atan2(dy, dx);
    double lambda = hypot(dx, dy) / 2.0;

    ScaledRobotPosition res;
    res.lambda = lambda;
    res.thetaStart = mod2pi(start.theta - phi);
    res.thetaEnd = mod2pi(end.theta - phi);
    res.kMax = kMax * lambda;

    return res;
}

