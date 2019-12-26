#ifndef STUDENT_PROJECT_GRAPH_H
#define STUDENT_PROJECT_GRAPH_H

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

using namespace std;

namespace student {
    class Graph {
    public:
        vector<pair<Point, Point>> edges;
    };
}

#endif
