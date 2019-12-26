#ifndef STUDENT_PROJECT_COLLISIONDETECTOR_H
#define STUDENT_PROJECT_COLLISIONDETECTOR_H


#include <utility>
#include <utils.hpp>

using namespace std;

namespace student {

    class Segment {
    public:
        Point a, b;

        Segment(const Point &a, const Point &b) : a(a), b(b) {}
    };

    class CollisionDetector {
    private:
        vector<Polygon> obstacles;
        int isPointInObstacle(const Point p, const Polygon& polygon);


    public:
        explicit CollisionDetector(vector<Polygon> obstacles) : obstacles(std::move(obstacles)) {}

        bool isPointInAnyObstacle(const Point &point);
    };
}


#endif
