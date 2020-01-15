#include "Mission1RTT.h"
#include "dubins/dubins.h"
#include "../DebugImage.h"

using namespace student;
using namespace std;




boost::optional<vector<DubinsCurve>> student::Mission1RTT::solve() {
    graph.addPoint(start);
    bool keepGoing = true;
    do {
        for (int i = 0; i < 100; i++) {
            auto p = getRandomPoint();

            //check if in obstacle
            if (!collisionDetector->isPointInAnyObstacle(p.toPoint())) {
                auto nearP = graph.nearestPoint(p);

                auto dubinsCurves = dubinsShortestPath(p, *nearP, 10);

                for (const auto &curve : dubinsCurves) {
                    if (!collisionDetector->doesCurveCollide(curve)) {
                        graph.addEdge(nearP, graph.addPoint(p));
                    }
                }
            }
        }
    } while(!allPointsReached());

    for (const auto&point : graph.points) {
        DebugImage::drawPoint(point->toPoint());
    }
    DebugImage::showAndWait();

    return boost::optional<vector<DubinsCurve>>();
}

RobotPosition Mission1RTT::getRandomPoint() {
    //random point
    //int t = (rand() % 7) + 1;
    float t =  static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 360));
    float LOx = 0, HIx = 1.5, LOy = 0, HIy= 1.0;
    float x = LOx + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HIx - LOx)));
    float y = LOy + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HIy - LOy)));
    cout << "Random point x = " << x << " y = " << y << " t = " << t << endl;
    return {x, y, t};
}

bool Mission1RTT::allPointsReached() {
    return true;
}

