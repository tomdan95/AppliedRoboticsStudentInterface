#include "Scaler.h"

using namespace std;
using namespace student;

// TODO: Remove duplication


vector<Polygon> Scaler::scaleToInt(const vector<Polygon> &polygons) {
    vector<Polygon> scaled;
    for (const auto &polygon:polygons) {
        scaled.push_back(scaleToInt(polygon));
    }
    return scaled;
}

Polygon Scaler::scaleToInt(const Polygon &polygon) {
    Polygon scaled;
    for (const auto &point:polygon) {
        scaled.push_back(scaleToInt(point));
    }
    return scaled;
}

Point Scaler::scaleToInt(const Point &point) {
    Point scaled(point.x * intRound, point.y * intRound);
    return scaled;
}


vector<Point> Scaler::rescaleBackToDouble(const vector<Point> &points) {
    Polygon scaled;
    for (const auto &point:points) {
        scaled.push_back(rescaleBackToDouble(point));
    }
    return scaled;
}

Point Scaler::rescaleBackToDouble(const Point &point) {
    Point scaled(point.x / intRound, point.y / intRound);
    return scaled;
}

