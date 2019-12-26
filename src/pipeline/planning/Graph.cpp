#include "Graph.h"

void student::Graph::connectTo(Point point) {
    Point *nearestPoint = this->getNearestPointTo(point);
    edges.emplace_back(nearestPoint, addPoint(point));
}

Point *student::Graph::getNearestPointTo(Point a) {
    double minDistance = 100000;
    Point *nearest = NULL;
    for (auto *b:points) {
        double distance = distanceBetween(a, *b);
        if (distance < minDistance) {
            minDistance = distance;
            nearest = b;
        }
    }
    return nearest;
}

double student::Graph::distanceBetween(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}


void student::Graph::addEdge(Point a, Point b) {
    edges.emplace_back(addPoint(a), addPoint(b));
}

Point *student::Graph::addPoint(Point p) {
    auto *copy = new Point(p.x, p.y);
    points.insert(copy);
    return copy;
}

