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

namespace student {

    Graph findCleanestPaths(const vector<Polygon> &obstaclesAndArena, CollisionDetector *collisionDetector) {
        vector<VoronoiSegment> segments = mapPolygonsToVoronoiSegments(obstaclesAndArena);
        voronoi_diagram<double> vd;
        construct_voronoi(segments.begin(), segments.end(), &vd);
        return getCleanestPathFromVoroniDiagram(vd, obstaclesAndArena, collisionDetector);
    }

    vector<VoronoiSegment> mapPolygonsToVoronoiSegments(const vector<Polygon> &obstaclesAndArena) {
        vector<VoronoiSegment> segments;
        for (const Polygon &obstacle:obstaclesAndArena) {
            Point start = obstacle[0];
            for (int i = 1; i < obstacle.size(); i++) {
                const Point end = obstacle[i];
                segments.emplace_back(start.x, start.y, end.x, end.y);
                start = end;
            }

            // Add last point to close the polygon
            const Point end = obstacle[0];
            segments.emplace_back(start.x, start.y, end.x, end.y);
        }
        return segments;
    }

    Graph getCleanestPathFromVoroniDiagram(const voronoi_diagram<double> &vd, const vector<Polygon> &obstaclesAndArena,
                                           CollisionDetector *collisionDetector) {
        Graph graph;
        for (auto edge : vd.edges()) {
            if (edge.is_primary() && edge.vertex0() && edge.vertex1()) {
                double startX = edge.vertex0()->x();
                double startY = edge.vertex0()->y();
                double endX = edge.vertex1()->x();
                double endY = edge.vertex1()->y();
                Point start(startX, startY);
                Point end(endX, endY);
                if (!collisionDetector->isPointInAnyObstacle(start) && !collisionDetector->isPointInAnyObstacle(end)) {
                    graph.addEdge(start, end);
                }
            }
        }
        return graph;
    }

    // TODO: Move
    void drawEdgeAndObstacles(const boost::polygon::voronoi_edge<double> edge, const vector<Polygon> obstacles) {
        cv::Mat image(1000, 1280, CV_8UC3, cv::Scalar(0, 0, 255));

        cv::line(image, cv::Point(edge.vertex0()->x(), edge.vertex0()->y()),
                 cv::Point(edge.vertex1()->x(), edge.vertex1()->y()), cv::Scalar(255, 255, 255), 5);

        for (const auto &obstacle:obstacles) {
            cv::Point start(obstacle[0].x, obstacle[0].y);
            for (int i = 1; i < obstacle.size(); i++) {
                cv::Point end(obstacle[i].x, obstacle[i].y);
                cv::line(image, start, end, cv::Scalar(255, 255, 0));
                start = end;
            }
            cv::Point end(obstacle[0].x, obstacle[0].y);
            cv::line(image, start, end, cv::Scalar(255, 0, 0));
        }
        showImageAndWaitKeyPress(image);
    }


}
