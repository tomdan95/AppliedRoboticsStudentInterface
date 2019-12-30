#ifndef STUDENT_PROJECT_DISCRETIZER_H
#define STUDENT_PROJECT_DISCRETIZER_H


#include <utils.hpp>

using namespace std;


namespace student {
    class Scaler {
    private:
        const double intRound;

    public:
        Scaler(const double intRound) : intRound(intRound) {}

        vector<Polygon> scaleToInt(const vector<Polygon> &polygons);
        Polygon scaleToInt(const Polygon &polygon);
        Point scaleToInt(const Point& point);

        vector<Point> rescaleBackToDouble(const vector<Point> &points);
        Point rescaleBackToDouble(const Point& point);

    };
}


#endif //STUDENT_PROJECT_DISCRETIZER_H
