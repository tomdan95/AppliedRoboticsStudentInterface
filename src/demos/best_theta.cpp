#include <vector>
#include <utils.hpp>
#include <iostream>
#include "../pipeline/planning/dubins/curve.h"
#include "../pipeline/planning/best_theta/best_theta.h"
#include "../pipeline/planning/dubins/dubins.h"
#include "../pipeline/DebugImage.h"

using namespace std;
using namespace student;

int main() {
    vector<Point *> path;

    path.push_back(new Point(0.1, 0.1));
    path.push_back(new Point(0.2, 0.1));
    path.push_back(new Point(0.5, 0.1));
    path.push_back(new Point(0.5, 1));
    path.push_back(new Point(0.2, 1));
    path.push_back(new Point(0.1, 1));

    vector<DubinsCurve> curves = findBestTheta(path, 0);


    cout << "discretizing..." << endl;
    vector<Pose> allPoses;
    for (auto curve:curves) {
        vector<Pose> poses = dubinsCurveToPoseVector(curve);
        allPoses.insert(allPoses.end(), poses.begin(), poses.end());
    }
    cout << "done" << endl;

    DebugImage::drawPoses(allPoses);
    DebugImage::drawPath(path);
    DebugImage::showAndWait();

    return 0;
}
