#ifndef DUBLINS_CURVE_H
#define DUBLINS_CURVE_H

#include "temp.h"
#include "primitives.h"



class DubinsArc {
public:
    double x0, y0, th0;
    double xf, yf, thf;
    double k, L;
};

class DubinsCurve {
public:
    DubinsArc a1, a2, a3;
    double L;
};


void circleLine(double s, double x0, double y0, double th0, double k, DubinsArc * result);
DubinsCurve dubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2);

//void dubinsArc(RobotPosition start, );

#endif //DUBLINS_CURVE_H
