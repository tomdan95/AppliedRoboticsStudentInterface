#include "utils.h"



double mod2pi(double angle) {
    double res = angle;
    while (res < 0) {
        res = res + 2 * M_PI;
    }
    while (res >= 2 * M_PI) {
        res = res - 2 * M_PI;
    }
    return res;
}

