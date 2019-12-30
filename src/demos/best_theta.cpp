#include <vector>
#include <utils.hpp>
#include <iostream>
#include "../pipeline/planning/best_theta/best_theta.h"
#include "../pipeline/planning/dubins/dubins.h"
#include "../pipeline/DebugImage.h"

using namespace std;
using namespace student;

int main() {
    vector<Point> path;

    path.emplace_back(0.1, 0.1);
    path.emplace_back(0.3, 0.1);
    path.emplace_back(0.6, 0.1);
    path.emplace_back(0.6, 1);
    path.emplace_back(0.3, 1);
    path.emplace_back(0.1, 1);

    vector<Polygon> obstacles;
    CollisionDetector detector(obstacles);
    vector<DubinsCurve> curves = findBestDubinsCurves(path, 0, &detector);


    cout << "discretizing..." << endl;
    vector<Pose> allPoses;
    for (auto curve:curves) {
        vector<Pose> poses = dubinsCurveToPoseVector(curve);
        allPoses.insert(allPoses.end(), poses.begin(), poses.end());
    }
    cout << "done" << endl;

    DebugImage::drawPoses(allPoses);
//    DebugImage::drawPath(path);
    DebugImage::showAndWait();

    return 0;
}
