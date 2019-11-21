#ifndef DUBLINS_PRIMITIVES_H
#define DUBLINS_PRIMITIVES_H

#include <math.h>
#include "scale.h"


class DubinsResult {
public:
    bool ok;
    double sc_s1, sc_s2, sc_s3;
    int pidx;

    double length();

    DubinsResult scaleFromStandard(double lambda);
};

class KSigns {
public:
    double k0, k1, k2;

    KSigns(double k0, double k1, double k2) {
        this->k0 = k0;
        this->k1 = k1;
        this->k2 = k2;
    }
};

class ManeuverSolver {
private:
    KSigns kSigns;
public:
    ManeuverSolver(double k0, double k1, double k2) : kSigns(k0, k1, k2) {}

    virtual DubinsResult solve(ScaledRobotPosition scale) = 0;

    KSigns getKSigns(double kMax) {
        return {
                this->kSigns.k0 * kMax,
                this->kSigns.k1 * kMax,
                this->kSigns.k2 * kMax
        };
    }
};


class LSL : public ManeuverSolver {
public:
    LSL() : ManeuverSolver(1, 0, 1) {}

    DubinsResult solve(ScaledRobotPosition scale) override;
};

class RSR : public ManeuverSolver {
public:
    RSR() : ManeuverSolver(-1, 0, -1) {}

    DubinsResult solve(ScaledRobotPosition scale) override;
};

class LSR : public ManeuverSolver {
public:
    LSR() : ManeuverSolver(1, 0, -1) {}

    DubinsResult solve(ScaledRobotPosition scale) override;
};

class RSL : public ManeuverSolver {
public:
    RSL() : ManeuverSolver(-1, 0, 1) {}

    DubinsResult solve(ScaledRobotPosition scale) override;
};

class RLR : public ManeuverSolver {
public:
    RLR() : ManeuverSolver(-1, 1, -1) {}

    DubinsResult solve(ScaledRobotPosition scale) override;
};

class LRL : public ManeuverSolver {
public:
    LRL() : ManeuverSolver(1, -1, 1) {}

    DubinsResult solve(ScaledRobotPosition scale) override;
};

#endif //DUBLINS_PRIMITIVES_H
