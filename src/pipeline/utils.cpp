#include "utils.h"


namespace student {
    Point getPolygonCenter(const Polygon &polygon) {
        Point center;
        for (auto point: polygon) {
            center.x += point.x;
            center.y += point.y;
        }
        center.x /= polygon.size();
        center.y /= polygon.size();
        return center;
    }
}