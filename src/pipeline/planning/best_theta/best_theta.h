#ifndef STUDENT_PROJECT_BEST_THETA_H
#define STUDENT_PROJECT_BEST_THETA_H

#include <utils.hpp>
#include "../dubins/curve.h"

using namespace std;

namespace student {
    vector<DubinsCurve> findBestTheta(vector<Point *> path, double robotTheta);
}

#endif
