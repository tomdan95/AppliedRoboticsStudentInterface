#include "Graph.h"
#include "../DebugImage.h"

Point *student::Graph::addAndConnectToNearNotCollidingPoints(Point point, const CollisionDetector *collisionDetector) {
    Point *copyOfPoint = findOrAddPoint(point);
    int addedTo = 0;
    double threshold = 0.3;
    while(addedTo <= 0) {
        for (auto *b:points) {
            double distance = distanceBetween(*copyOfPoint, *b);
            if (distance < threshold) { // TODO: add threshold variable
                //TODO: Check if collide
                //TODO: If not collide add point
                if(!collisionDetector->doesSegmentCollide(point, *b)) {
                    addEdge(b, copyOfPoint);
                    addedTo++;
                }
            }
        }
        threshold += 0.1;
    }
    return copyOfPoint;
}


double student::Graph::distanceBetween(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}


void student::Graph::addEdge(Point a, Point b) {
    addEdge(findOrAddPoint(a), findOrAddPoint(b));
}

void student::Graph::addEdge(Point *a, Point *b) {
    edges[a].push_back(b);
    edges[b].push_back(a);
}

Point *student::Graph::findOrAddPoint(Point p) {
    for (auto *a : points) {
        if (a->x == p.x && a->y == p.y) {
            return a;
        }
    }
    auto *copy = new Point(p.x, p.y);
    points.insert(copy);
    return copy;
}

// Distance must be kept as first item to keep sorting
// TODO: Explain better
typedef pair<double, Point *> PointWithDistance;

vector<Point *> student::Graph::shortestPathFromTo(Point *from, Point *to, const vector<Point *>& disadvantage) {
    map<Point *, Point *> predecessorOf;
    predecessorOf[from] = NULL;

    priority_queue<PointWithDistance, vector<PointWithDistance>, greater<PointWithDistance>> toVisit;
    map<Point *, double> distances;
    for (auto *p : points) {
        distances[p] = INFINITY;
    }

    toVisit.push(make_pair(0, from));
    distances[from] = 0;

    while (predecessorOf[to] == NULL && !toVisit.empty()) {
        auto *u = toVisit.top().second;
        toVisit.pop();

        vector<Point *> adjacentPoints = edges[u];
        for (auto *v:adjacentPoints) {
            double weight = distanceBetween(*u, *v);

            for (auto pointToAvoid : disadvantage) {
                if (pointToAvoid == v) {
                    weight += 1.0;
                    cout << "[DIJKSTRA] weight increased to avoid back path" << endl;
                    break;
                }
            }

            if (distances[v] > distances[u] + weight) {
                distances[v] = distances[u] + weight;
                predecessorOf[v] = u;
                toVisit.push(make_pair(distances[v], v));
            }
        }
    }

    vector<Point *> path;
    Point *predecessor = to;
    while (predecessor != NULL) {
        path.insert(path.begin(), predecessor);
        predecessor = predecessorOf[predecessor];
    }
    return path;
}

