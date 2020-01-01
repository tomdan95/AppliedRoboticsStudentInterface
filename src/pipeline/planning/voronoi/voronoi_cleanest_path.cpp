#include <vector>

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

    Graph findCleanestPaths(const vector<Polygon> &obstaclesAndArena, CollisionDetector*collisionDetector) {
        vector<VoronoiSegment> segments = mapPolygonsToVoronoiSegments(obstaclesAndArena);
        voronoi_diagram<double> vd;
        construct_voronoi(segments.begin(), segments.end(), &vd);
        return getCleanestPathFromVoroniDiagram(vd, obstaclesAndArena, collisionDetector);
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
                                           CollisionDetector*collisionDetector) {
        Graph graph;
        for (auto edge : vd.edges()) {
            if (edge.is_primary() && edge.vertex0() && edge.vertex1()) {
                double startX = edge.vertex0()->x() / VORONOI_DOUBLE_TO_INT;
                double startY = edge.vertex0()->y() / VORONOI_DOUBLE_TO_INT;
                double endX = edge.vertex1()->x() / VORONOI_DOUBLE_TO_INT;
                double endY = edge.vertex1()->y() / VORONOI_DOUBLE_TO_INT;
                Point start(startX, startY);
                Point end(endX, endY);
                if (!collisionDetector->isPointInAnyObstacle(start) && !collisionDetector->isPointInAnyObstacle(end)) {
                    graph.addEdge(start, end);
                }
            }
        }
        return graph;
    }



}
