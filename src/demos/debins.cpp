#include <iostream>
#include "../pipeline/planning/dubins/primitives.h"
#include "../pipeline/planning/dubins/dubins.h"

using namespace std;


int main() {

    //robot x = 0.19305 y = 0.2249 theta = 0.00641015
    //gate x = 1.20315 y = 0.94575 theta = 0
    RobotPosition start(0.19305, 0.2249,0.00641015);
    RobotPosition end(1.20315, 0.94575, 0);
    double kMax = 10;

    auto res = dubinsShortestPath(start, end, kMax)[0];

    cout << "Res: " << endl;
    cout << "Arc 1 xf=" << res.a1.xf << " yf=" << res.a1.yf << " thf=" << res.a1.thf << endl;
    cout << "Arc 2 xf=" << res.a2.xf << " yf=" << res.a2.yf << " thf=" << res.a2.thf << endl;
    cout << "Arc 3 x0=" << res.a3.x0 << " y0=" << res.a3.y0 << " th0=" << res.a3.th0 << endl;
    cout << "Arc 3 xf=" << res.a3.xf << " yf=" << res.a3.yf << " thf=" << res.a3.thf << endl;
    cout << "Length=" << res.length() << endl;
    return 0;
}


