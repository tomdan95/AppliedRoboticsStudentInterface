// Boost.Polygon library voronoi_basic_tutorial.cpp file

//          Copyright Andrii Sydorchuk 2010-2012.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// See http://www.boost.org for updates, documentation, and revision history.

#include <cstdio>
#include <vector>
#include <iostream>

#include "../../opencv-utils.h"

#include <boost/polygon/voronoi.hpp>
#include "voronoi_helper.h"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

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
    }  // polygon
}  // boost

void iterate_primary_edges1(const voronoi_diagram<double> &vd, cv::Mat &rgbImage) {

    int i = 0;
    for (auto edge : vd.edges()) {
        if (edge.is_primary() && edge.vertex0() && edge.vertex1()) {

            double startX = edge.vertex0()->x();
            double startY = edge.vertex0()->y();
            double endX = edge.vertex1()->x();
            double endY = edge.vertex1()->y();

            cv::Point start(startX, startY);
            cv::Point end(endX, endY);
            cv::line(rgbImage, start, end, cv::Scalar(0, 255, 0), 3);
        }
    }
    std::cout << "fatto" << endl;

}

int testComputeVoronoi(cv::Mat &image, vector<Polygon> &vector) {
    // Preparing Input Geometries.

    std::vector<Segment> segments;

    for (const Polygon &obstacle:vector) {
        cv::Point start(obstacle[0].x * 1000.0, obstacle[0].y * 1000.0);
        for (int i = 1; i < obstacle.size(); i++) {
            const Point point = obstacle[i];

            cv::Point end(point.x * 1000.0, point.y * 1000.0);
            segments.emplace_back(start.x, start.y, end.x, end.y);

            start = end;
        }

        // add last
        cv::Point end(obstacle[0].x * 1000.0, obstacle[0].y * 1000.0);
        segments.emplace_back(start.x, start.y, end.x, end.y);
    }




    std::vector<VoronoiPoint> points;

/*
    segments.push_back(Segment(100, 100, 100, 200));
    segments.push_back(Segment(100, 200, 200, 200));
    segments.push_back(Segment(200, 200, 200, 100));
    segments.push_back(Segment(200, 100, 100, 100));
*/
    segments.emplace_back(10, 10, 1200, 10);
    segments.emplace_back(1200, 10, 1200, 900);
    segments.emplace_back(1200, 900, 10, 900);
    segments.emplace_back(10, 900, 10, 10);


    // Draw segments
    for (const auto segment : segments) {
        auto start = cv::Point(segment.p0.a, segment.p0.b);
        auto end = cv::Point(segment.p1.a, segment.p1.b);
        cv::circle(image, start, 10, cv::Scalar(100, 100, 0), 3);
        cv::circle(image, end, 10, cv::Scalar(100, 100, 0), 3);
        cv::line(image, start, end, cv::Scalar(255, 0, 0), 3);
    }

    // Construction of the Voronoi Diagram.
    voronoi_diagram<double> vd;

    std::cout << "before construct voronoi" << std::endl;
    construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);
    std::cout << "after construct voronoi" << std::endl;

    std::cout << "before iterate" << std::endl;
    iterate_primary_edges1(vd, image);
    std::cout << "after iterate" << std::endl;
    showImageAndWaitKeyPress(image);
    return 0;
}
