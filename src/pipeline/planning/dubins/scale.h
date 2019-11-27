#ifndef DUBLINS_SCALE_H
#define DUBLINS_SCALE_H

#include <math.h>
#include "utils.h"
#include "temp.h"

class ScaledRobotPosition {
public:
    double thetaStart, thetaEnd, kMax;
    double lambda;
};


ScaledRobotPosition scaleToStandard(RobotPosition start, RobotPosition end, double kMax);


#endif //DUBLINS_SCALE_H
