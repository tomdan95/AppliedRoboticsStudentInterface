#ifndef STUDENT_PROJECT_INFLATE_H
#define STUDENT_PROJECT_INFLATE_H

#include "student_planning_interface.hpp"

vector<Polygon> inflateObstacles(const vector<Polygon> &obstacles, const Polygon &borders, int robotSize);


/*
namespace student {

    class Inflator {
    private:
        double robotSize;
    public:
        explicit Inflator(double robotSize) : robotSize(robotSize) {}

        vector<Polygon> inflateObstacles(const vector<Polygon> &obstacles);

        Polygon deflateArenaBorders(const Polygon &borders);
    };

    vector<Polygon>
    mergeInflatedObstaclesWithDeflatedArenaBorders(const vector<Polygon> &inflatedObstacles, const Polygon &borders);

}*/

#endif //STUDENT_PROJECT_INFLATE_H
