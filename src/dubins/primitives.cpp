#include "primitives.h"

double DubinsCurve::length() {
    return sc_s1 + sc_s2 + sc_s3;
}

DubinsCurve DubinsCurve::scaleFromStandard(double lambda) {
    DubinsCurve res;
    res.ok = ok;
    res.sc_s1 = sc_s1 * lambda;
    res.sc_s2 = sc_s2 * lambda;
    res.sc_s3 = sc_s3 * lambda;
    return res;
}

DubinsCurve LSL::solve(ScaledRobotPosition scale) {
    double invK = 1.0 / scale.kMax;
    double C = cos(scale.thetaEnd) - cos(scale.thetaStart);
    double S = 2.0 * scale.kMax + sin(scale.thetaStart) - sin(scale.thetaEnd);
    double temp1 = atan2(C, S);
    double sc_s1 = invK * mod2pi(temp1 - scale.thetaStart);
    double temp2 = 2.0 + 4.0 * pow(scale.kMax, 2) - 2 * cos(scale.thetaStart - scale.thetaEnd) +
                   4.0 * scale.kMax * (sin(scale.thetaStart) - sin(scale.thetaEnd));

    DubinsCurve res;
    if (temp2 < 0) {
        res.ok = false;
    } else {
        res.ok = true;
        res.sc_s1 = sc_s1;
        res.sc_s2 = invK * sqrt(temp2);
        res.sc_s3 = invK * mod2pi(scale.thetaEnd - temp1);
    }
    return res;
}

DubinsCurve RSR::solve(ScaledRobotPosition scale) {
    double invK = 1.0 / scale.kMax;
    double C = cos(scale.thetaStart) - cos(scale.thetaEnd);
    double S = 2 * scale.kMax - sin(scale.thetaStart) + sin(scale.thetaEnd);
    double temp1 = atan2(C, S);
    double sc_s1 = invK * mod2pi(scale.thetaStart - temp1);
    double temp2 = 2 + 4 * pow(scale.kMax, 2) - 2 * cos(scale.thetaStart - scale.thetaEnd) -
                   4 * scale.kMax * (sin(scale.thetaStart) - sin(scale.thetaEnd));
    DubinsCurve res;
    if (temp2 < 0) {
        res.ok = false;
    } else {
        res.ok = false;
        res.sc_s1 = sc_s1;
        res.sc_s2 = invK * sqrt(temp2);
        res.sc_s3 = invK * mod2pi(temp1 - scale.thetaEnd);
    }
    return res;
}

DubinsCurve LSR::solve(ScaledRobotPosition scale) {
    double invK = 1.0 / scale.kMax;
    double C = cos(scale.thetaStart) + cos(scale.thetaEnd);
    double S = 2 * scale.kMax + sin(scale.thetaStart) + sin(scale.thetaEnd);
    double temp1 = atan2(-C, S);
    double temp3 = 4 * pow(scale.kMax, 2) - 2 + 2 * cos(scale.thetaStart - scale.thetaEnd) +
                   4 * scale.kMax * (sin(scale.thetaStart) + sin(scale.thetaEnd));
    DubinsCurve res;
    if (temp3 < 0) {
        res.ok = false;
    } else {
        res.ok = false;
        double sc_s2 = invK * sqrt(temp3);
        double temp2 = -atan2(-2, sc_s2 * scale.kMax);
        res.sc_s1 = invK * mod2pi(temp1 + temp2 - scale.thetaStart);
        res.sc_s2 = sc_s2;
        res.sc_s3 = invK * mod2pi(temp1 + temp2 - scale.thetaEnd);
    }
    return res;
}

DubinsCurve RSL::solve(ScaledRobotPosition scale) {
    double invK = 1.0 / scale.kMax;
    double C = cos(scale.thetaStart) + cos(scale.thetaEnd);
    double S = 2 * scale.kMax - sin(scale.thetaStart) - sin(scale.thetaEnd);
    double temp1 = atan2(C, S);
    double temp3 = 4 * pow(scale.kMax, 2) - 2 + 2 * cos(scale.thetaStart - scale.thetaEnd) -
                   4 * scale.kMax * (sin(scale.thetaStart) + sin(scale.thetaEnd));
    DubinsCurve res;
    if (temp3 < 0) {
        res.ok = false;
    } else {
        res.ok = false;
        double sc_s2 = invK * sqrt(temp3);
        double temp2 = atan2(2, sc_s2 * scale.kMax);
        res.sc_s1 = invK * mod2pi(scale.thetaStart - temp1 + temp2);
        res.sc_s2 = sc_s2;
        res.sc_s3 = invK * mod2pi(scale.thetaEnd - temp1 + temp2);
    }
    return res;
}

DubinsCurve RLR::solve(ScaledRobotPosition scale) {
    double invK = 1.0 / scale.kMax;
    double C = cos(scale.thetaStart) - cos(scale.thetaEnd);
    double S = 2 * scale.kMax - sin(scale.thetaStart) + sin(scale.thetaEnd);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(scale.kMax, 2) + 2 * cos(scale.thetaStart - scale.thetaEnd) +
                                                 4 * scale.kMax * (sin(scale.thetaStart) - sin(scale.thetaEnd)));
    DubinsCurve res;
    if (abs(temp2) > 1) {
        res.ok = false;
    } else {
        res.ok = true;
        double sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
        double sc_s1 = invK * mod2pi(scale.thetaStart - temp1 + 0.5 * sc_s2 * scale.kMax);
        res.sc_s1 = sc_s1;
        res.sc_s2 = sc_s2;
        res.sc_s3 = invK * mod2pi(scale.thetaStart - scale.thetaEnd + scale.kMax * (sc_s2 - sc_s1));
    }
    return res;
}

DubinsCurve LRL::solve(ScaledRobotPosition scale) {
    double invK = 1.0 / scale.kMax;
    double C = cos(scale.thetaEnd) - cos(scale.thetaStart);
    double S = 2 * scale.kMax + sin(scale.thetaStart) - sin(scale.thetaEnd);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * (6 - 4 * pow(scale.kMax, 2) + 2 * cos(scale.thetaStart - scale.thetaEnd) -
                                                 4 * scale.kMax * (sin(scale.thetaStart) - sin(scale.thetaEnd)));
    DubinsCurve res;
    if (abs(temp2) > 1) {
        res.ok = false;
    } else {
        res.ok = true;
        double sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
        double sc_s1 = invK * mod2pi(temp1 - scale.thetaStart + 0.5 * sc_s2 * scale.kMax);
        res.sc_s1 = sc_s1;
        res.sc_s2 = sc_s2;
        res.sc_s3 = invK * mod2pi(scale.thetaEnd - scale.thetaStart + scale.kMax * (sc_s2 - sc_s1));
    }
    return res;
}

