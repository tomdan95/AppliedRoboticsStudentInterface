#include <vector>
#include <utils.hpp>
#include <iostream>
#include "../pipeline/planning/best_theta/BestThetaFinder.h"
#include "../pipeline/planning/dubins/dubins.h"
#include "../pipeline/DebugImage.h"
#include "../pipeline/planning/collision_detection/ShadowCollisionDetector.h"

using namespace std;
using namespace student;

int main() {
    vector<Point *> path;

    path.push_back(new Point(0.1, 0.1));
    path.push_back(new Point(0.3, 0.1));
    path.push_back(new Point(0.6, 0.1));
    path.push_back(new Point(0.6, 1));
    path.push_back(new Point(0.3, 1));
    path.push_back(new Point(0.1, 1));

    vector<Polygon> obstacles;
    vector<Point> border;
    Polygon gate;
    vector<Polygon> victims;
    ShadowCollisionDetector detector(border, obstacles, gate, victims);

    BestThetaFinder finder(10, 0, &detector);
    if(auto curves = finder.findBestDubinsCurves(path)) {

        vector<Pose> allPoses;
        for (auto curve:*curves) {
            vector<Pose> poses = dubinsCurveToPoseVector(curve);
            allPoses.insert(allPoses.end(), poses.begin(), poses.end());
        }

        DebugImage::drawPoses(allPoses);
        DebugImage::drawPath(path);
        DebugImage::showAndWait();
    } else {
        cout << "didn't find a solution" << endl;
    }

    return 0;
}
