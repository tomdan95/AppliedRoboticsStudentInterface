#ifndef STUDENT_PROECT_VORONOI_CLEANEST_PATH_H
#define STUDENT_PROECT_VORONOI_CLEANEST_PATH_H

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "../Graph.h"
#include <boost/polygon/voronoi.hpp>

using boost::polygon::voronoi_diagram;

struct VoronoiPoint {
    int x;
    int y;

    VoronoiPoint(int x, int y) : x(x), y(y) {}
};

struct VoronoiSegment {
    VoronoiPoint p0;
    VoronoiPoint p1;

    VoronoiSegment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};


namespace student {
    Graph findCleanestPaths(const vector<Polygon> &obstaclesAndArena, const vector<Polygon> &obstacles);

    // private functions  (TODO: Consider creating a class to encapsulate them)
    vector<VoronoiSegment>
    mapPolygonsToVoronoiSegments(const vector<Polygon> &obstaclesAndArena);

    Graph getCleanestPathFromVoroniDiagram(const voronoi_diagram<double> &vd, const vector<Polygon> &obstaclesAndArena, const vector<Polygon> &obstacles);
    bool isEdgeInsideObstacle(const boost::polygon::voronoi_edge<double> edge, const vector<Polygon> &obstacles);
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
                return (orient == HORIZONTAL) ? point.x : point.y;
            }
        };

        template<>
        struct geometry_concept<VoronoiSegment> {
            typedef segment_concept type;
        };

        template<>
        struct segment_traits<VoronoiSegment> {
            typedef int coordinate_type;
            typedef VoronoiPoint point_type;

            static inline point_type get(const VoronoiSegment &segment, direction_1d dir) {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };
    }
}


#endif //STUDENT_PROECT_VORONOI_CLEANEST_PATH_H
