#include <cstdlib>
#include "models.h"
#include "utils.h"


StandardDubinsProblem scaleToStandard(RobotPosition start, RobotPosition end, double kMax) {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double phi = atan2(dy, dx);
    double lambda = hypot(dx, dy) / 2.0;
    double thetaStart = mod2pi(start.theta - phi);
    double thetaEnd = mod2pi(end.theta - phi);
    return {
            thetaStart,
            thetaEnd,
            kMax * lambda,
            lambda
    };
}

DubinsArc dubinsArc(double x0, double y0, double th0, double k, double L) {
    DubinsArc result;
    result.x0 = x0;
    result.y0 = y0;
    result.th0 = th0;
    result.k = k;
    result.length = L;
    circleLine(L, x0, y0, th0, k, &result);
    return result;
}


DubinsCurve
dubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2) {
    DubinsCurve result;
    result.a1 = dubinsArc(x0, y0, th0, k0, s1);
    result.a2 = dubinsArc(result.a1.xf, result.a1.yf, result.a1.thf, k1, s2);
    result.a3 = dubinsArc(result.a2.xf, result.a2.yf, result.a2.thf, k2, s3);
    return result;
}


// TODO: Rename
void circleLine(double s, double x0, double y0, double th0, double k, DubinsArc *result) {
    result->xf = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2.0);
    result->yf = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2.0);
    result->thf = mod2pi(th0 + k * s);
}

double sinc(double t) {
    if (abs(t) < 0.002) {
        return 1.0 - pow(t, 2.0) / 6. * (1 - pow(t, 2.0) / 20.0);
    } else {
        return sin(t) / t;
    }
}


