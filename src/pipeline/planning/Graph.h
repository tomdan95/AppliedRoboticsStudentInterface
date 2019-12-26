#ifndef STUDENT_PROJECT_GRAPH_H
#define STUDENT_PROJECT_GRAPH_H

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

using namespace std;

namespace student {
    class Graph {
    public:
        vector<pair<Point*, Point*>> edges;
        set<Point*> points;

        void addEdge(Point a, Point b);

        void connectTo(Point point);

    private:
        Point* addPoint(Point p);
        Point* getNearestPointTo(Point point);

        static double distanceBetween(Point a, Point b);
    };
}

#endif
