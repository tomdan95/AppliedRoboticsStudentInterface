#ifndef STUDENT_PROECT_VORONOI_CLEANEST_PATH_H
#define STUDENT_PROECT_VORONOI_CLEANEST_PATH_H

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "Graph.h"
#include <boost/polygon/voronoi.hpp>

using boost::polygon::voronoi_diagram;

struct VoronoiPoint {
    int a;
    int b;

    VoronoiPoint(int x, int y) : a(x), b(y) {}
};

struct Segment {
    VoronoiPoint p0;
    VoronoiPoint p1;

    Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};



namespace student {
    Graph findCleanestPaths(vector<Polygon> &obstaclesAndArena);

    // private functions  (TODO: Consider creating a class to encapsulate them)
    vector<Segment> mapPolygonsToVoronoiSegments(const vector<Polygon> &obstaclesAndArena);
    Graph getCleanestPathFromVoroniDiagram(const voronoi_diagram<double> &vd, vector<Polygon> &obstacles);
    bool isPointOverObstacle(double xb, double yb, vector<Polygon> &obstacles);
    bool isSimilar (double a, double b);
}

namespace boost {
    namespace polygon {

        template<>
        struct geometry_concept<VoronoiPoint> {
            typedef point_concept type;
        };

        template<>
        struct point_traits<VoronoiPoint> {
            typedef int coordinate_type;

            static inline coordinate_type get(
                    const VoronoiPoint &point, orientation_2d orient) {
                return (orient == HORIZONTAL) ? point.a : point.b;
            }
        };

        template<>
        struct geometry_concept<Segment> {
            typedef segment_concept type;
        };

        template<>
        struct segment_traits<Segment> {
            typedef int coordinate_type;
            typedef VoronoiPoint point_type;

            static inline point_type get(const Segment &segment, direction_1d dir) {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };
    }
}


#endif //STUDENT_PROECT_VORONOI_CLEANEST_PATH_H
