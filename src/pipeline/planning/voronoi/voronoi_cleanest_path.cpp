// Boost.Polygon library voronoi_basic_tutorial.cpp file

//          Copyright Andrii Sydorchuk 2010-2012.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// See http://www.boost.org for updates, documentation, and revision history.

#include <cstdio>
#include <vector>
#include <iostream>

#include "../../../opencv-utils.h"
#include "voronoi_cleanest_path.h"
#include "../collision_detection/CollisionDetector.h"

using boost::polygon::voronoi_builder;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

#define VORONOI_DOUBLE_TO_INT 800.0

namespace student {

    Graph findCleanestPaths(const vector<Polygon> &obstaclesAndArena, const vector<Polygon> &obstacles) {
        vector<VoronoiSegment> segments = mapPolygonsToVoronoiSegments(obstaclesAndArena);
        voronoi_diagram<double> vd;
        construct_voronoi(segments.begin(), segments.end(), &vd);
        return getCleanestPathFromVoroniDiagram(vd, obstaclesAndArena, obstacles);
    }

    int coordinateToVornoi(double x) {
        return x * VORONOI_DOUBLE_TO_INT;
    }

    vector<VoronoiSegment> mapPolygonsToVoronoiSegments(const vector<Polygon> &obstaclesAndArena) {
        vector<VoronoiSegment> segments;
        for (const Polygon &obstacle:obstaclesAndArena) {
            Point start = obstacle[0];
            for (int i = 1; i < obstacle.size(); i++) {
                const Point end = obstacle[i];
                segments.emplace_back(coordinateToVornoi(start.x), coordinateToVornoi(start.y),
                                      coordinateToVornoi(end.x), coordinateToVornoi(end.y));
                start = end;
            }

            // Add last point to close the polygon
            const Point end = obstacle[0];
            segments.emplace_back(coordinateToVornoi(start.x), coordinateToVornoi(start.y),
                                  coordinateToVornoi(end.x), coordinateToVornoi(end.y));
        }
        return segments;
    }

    Graph getCleanestPathFromVoroniDiagram(const voronoi_diagram<double> &vd, const vector<Polygon> &obstaclesAndArena,
                                           const vector<Polygon> &obstacles) {
        Graph graph;
        for (auto edge : vd.edges()) {
            if (edge.is_primary() && edge.vertex0() && edge.vertex1()) {
                if (!isEdgeInsideObstacle(edge, obstacles)) {
                    // TODO: include in the resulting graph
                    double startX = edge.vertex0()->x();
                    double startY = edge.vertex0()->y();
                    double endX = edge.vertex1()->x();
                    double endY = edge.vertex1()->y();
                    // TODO: Back to double
                    graph.edges.emplace_back(Point(startX, startY), Point(endX, endY));
                }
            }
        }
        return graph;
    }

    void drawEdgeAndObstacles(const boost::polygon::voronoi_edge<double> edge, const vector<Polygon> obstacles) {
        cv::Mat image(1000, 1280, CV_8UC3, cv::Scalar(0, 0, 255));

        cv::line(image, cv::Point(edge.vertex0()->x(), edge.vertex0()->y()), cv::Point(edge.vertex1()->x(), edge.vertex1()->y()), cv::Scalar(255, 255, 255), 5);

        for (const auto&obstacle:obstacles) {
            cv::Point start(obstacle[0].x * VORONOI_DOUBLE_TO_INT, obstacle[0].y * VORONOI_DOUBLE_TO_INT);
            for(int i = 1; i < obstacle.size(); i++) {
                cv::Point end(obstacle[i].x * VORONOI_DOUBLE_TO_INT, obstacle[i].y * VORONOI_DOUBLE_TO_INT);
                cv::line(image, start, end, cv::Scalar(255, 255, 0));
                start = end;
            }
            cv::Point end(obstacle[0].x * VORONOI_DOUBLE_TO_INT, obstacle[0].y * VORONOI_DOUBLE_TO_INT);
            cv::line(image, start, end, cv::Scalar(255, 0, 0));
        }
        showImageAndWaitKeyPress(image);
    }

    bool isEdgeInsideObstacle(const boost::polygon::voronoi_edge<double> edge, const vector<Polygon>& obstacles) {
        CollisionDetector detector(obstacles);

        Point a(((double)edge.vertex0()->x()) / VORONOI_DOUBLE_TO_INT, ((double)edge.vertex0()->y()) / VORONOI_DOUBLE_TO_INT);
        Point b(((double)edge.vertex1()->x()) / VORONOI_DOUBLE_TO_INT, ((double)edge.vertex1()->y()) / VORONOI_DOUBLE_TO_INT);

        return detector.isPointInAnyObstacle(a) || detector.isPointInAnyObstacle(b);
    }



}
