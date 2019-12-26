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
#include "voronoi_cleanest_path.h"

using boost::polygon::voronoi_builder;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

#define VORONOI_DOUBLE_TO_INT 800.0

namespace student {

    Graph findCleanestPaths(vector<Polygon> &obstaclesAndArena) {
        vector<Segment> segments = mapPolygonsToVoronoiSegments(obstaclesAndArena);
        voronoi_diagram<double> vd;
        construct_voronoi(segments.begin(), segments.end(), &vd);
        return getCleanestPathFromVoroniDiagram(vd, obstaclesAndArena);
    }

    vector<Segment> mapPolygonsToVoronoiSegments(const vector<Polygon> &obstaclesAndArena) {
        vector<Segment> segments;

        for (const Polygon &obstacle:obstaclesAndArena) {
            cv::Point start(obstacle[0].x * VORONOI_DOUBLE_TO_INT, obstacle[0].y * VORONOI_DOUBLE_TO_INT);
            for (int i = 1; i < obstacle.size(); i++) {
                const Point point = obstacle[i];

                cv::Point end(point.x * VORONOI_DOUBLE_TO_INT, point.y * VORONOI_DOUBLE_TO_INT);
                segments.emplace_back(start.x, start.y, end.x, end.y);

                start = end;
            }

            // add last
            cv::Point end(obstacle[0].x * VORONOI_DOUBLE_TO_INT, obstacle[0].y * VORONOI_DOUBLE_TO_INT);
            segments.emplace_back(start.x, start.y, end.x, end.y);
        }
        return segments;
    }

    Graph getCleanestPathFromVoroniDiagram(const voronoi_diagram<double> &vd, vector<Polygon> &obstacles) {

        Graph graph;
        for (auto edge : vd.edges()) {
            if (edge.is_primary() && edge.vertex0() && edge.vertex1()) {
                double startX = edge.vertex0()->x();
                double startY = edge.vertex0()->y();
                double endX = edge.vertex1()->x();
                double endY = edge.vertex1()->y();

                if (!isPointOverObstacle(startX, startY, obstacles) && !isPointOverObstacle(endX, endY, obstacles)) {
                    // TODO: include in the resulting graph
                    graph.edges.emplace_back(Point(startX, startY), Point(endX, endY));
                }
            }
        }
        //showImageAndWaitKeyPress(image);
        return graph;
    }


    bool isPointOverObstacle(double xb, double yb, vector<Polygon> &obstacles) {
        // TODO: Naive implementation, replace with efficient one
        for (const auto &obstacle : obstacles) {
            for (const auto vertex : obstacle) {
                double xa = vertex.x * VORONOI_DOUBLE_TO_INT;
                double ya = vertex.y * VORONOI_DOUBLE_TO_INT;
                if (isSimilar(xa, xb) && isSimilar(ya, yb)) {
                    return true;
                }
            }
        }
        return false;
    }

    bool isSimilar(double a, double b) {
        return b > (a - 10.0) && b < (a + 10.0);
    }
}
