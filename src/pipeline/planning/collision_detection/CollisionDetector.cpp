
#include <iostream>
#include "CollisionDetector.h"
#include "../../DebugImage.h"

using namespace std;
using namespace student;

bool CollisionDetector::isPointInAnyObstacle(const Point &point) {
    for (const auto &obstacle:obstacles) {
        if (isPointInObstacle(point, obstacle)) {
            return true;
        }
    }
    return false;
}


int CollisionDetector::isPointInObstacle(const Point p, const Polygon &polygon) {
    int crossingCount = 0;

    Polygon copy = polygon;
    copy.push_back(polygon[0]);

    for (int i = 0; i < polygon.size(); i++) {
        Point a = copy[i];
        Point b = copy[i + 1];

        if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            crossingCount++;
        }
    }
    return (crossingCount % 2 != 0);

}
