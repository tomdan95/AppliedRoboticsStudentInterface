#include <iostream>
#include <cmath>
#include "scale.h"
#include "temp.h"
#include "primitives.h"
#include "curve.h"
#include "dubins.h"

using namespace std;


int main() {
    RobotPosition start(0, 0, (-2.0 / 3.0 * M_PI));
    RobotPosition end(4, 0, (M_PI / 3.0));
    double kMax = 3;

    auto res = dubinsShortestPath(start, end, kMax);

    cout << "Res: " << endl;
    cout << "Arc 1 xf=" << res.a1.xf << " yf=" << res.a1.yf << " thf=" << res.a1.thf << endl;
    cout << "Arc 2 xf=" << res.a2.xf << " yf=" << res.a2.yf << " thf=" << res.a2.thf << endl;
    cout << "Arc 3 xf=" << res.a3.xf << " yf=" << res.a3.yf << " thf=" << res.a3.thf << endl;
    cout << "Length=" << res.L << endl;
    return 0;
}


