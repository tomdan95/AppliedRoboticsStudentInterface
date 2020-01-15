#ifndef STUDENT_PROJECT_GRAPH_H
#define STUDENT_PROJECT_GRAPH_H

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "collision_detection/CollisionDetector.h"

using namespace std;

namespace student {

    class Graph {
    public:
        map<Point*, vector<Point*>> edges;
        set<Point*> points;

        void addEdge(Point a, Point b);
        void addEdge(Point* a, Point* b);

        /**
         * @param point point to add
         * @return Pointer to the copy of the new added point
         */
        Point * addAndConnectToNearNotCollidingPoints(Point point, const CollisionDetector *collisionDetector);

        vector<Point *> shortestPathFromTo(Point *a, Point *b, const vector<Point *>& disadvantage);

        static double distanceBetween(Point a, Point b);

    private:
        Point* findOrAddPoint(Point p);
        Point* getNearestPointTo(Point point);
    };
}

#endif
