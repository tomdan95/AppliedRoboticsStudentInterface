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

        /**
         * @param point point to add
         * @return Pointer to the copy of the new added point
         */
        Point * addAndConnectToNearestPoint(Point point);

        vector<Point *> shortestPathFromTo(Point *a, Point *b);

    private:
        Point* findOrAddPoint(Point p);
        Point* getNearestPointTo(Point point);

        static double distanceBetween(Point a, Point b);

        vector<Point *> getAdjacentPoints(Point *pPoint);
    };
}

#endif
