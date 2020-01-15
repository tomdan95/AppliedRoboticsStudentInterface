#ifndef STUDENT_PROJECT_MODELS_H
#define STUDENT_PROJECT_MODELS_H

#include <utils.hpp>

class RobotPosition {

public:
    double x, y;
    double theta;

public:
    RobotPosition(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    RobotPosition(Point p, double theta) : x(p.x), y(p.y), theta(theta) {}

    Point toPoint() {
        return {(float) x, (float) y};
    }
};


class StandardDubinsProblem {
public:
    double thetaStart, thetaEnd, kMax;
    double lambda;

    StandardDubinsProblem(double thetaStart, double thetaEnd, double kMax, double lambda) : thetaStart(thetaStart),
                                                                                            thetaEnd(thetaEnd),
                                                                                            kMax(kMax),
                                                                                            lambda(lambda) {}
};


StandardDubinsProblem scaleToStandard(RobotPosition start, RobotPosition end, double kMax);


class DubinsArc {
public:
    double x0, y0, th0;
    double xf, yf, thf;
    double k;
    double length;
};

class DubinsCurve {
public:
    DubinsArc a1, a2, a3;

    double length() {
        return a1.length + a2.length + a3.length;
    }

};


void circleLine(double s, double x0, double y0, double th0, double k, DubinsArc *result);

DubinsCurve
dubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2);

double sinc(double t);

#endif
