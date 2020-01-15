#include "ShadowCollisionDetector.h"
#include "../planning.h"
#include "../../../opencv-utils.h"
#include "../Graph.h"
#include "../../DebugImage.h"

using namespace std;
using namespace student;

#define OBSTACLES_MATRIX_SIDE 1000
#define POINT_DISCRETIZATION  500.0


ShadowCollisionDetector::ShadowCollisionDetector(
        const Polygon &borders,
        const vector<Polygon> &obstacles,
        const Polygon &gate,
        const vector<Polygon> &victims
) : obstaclesShadow(
        OBSTACLES_MATRIX_SIDE,
        OBSTACLES_MATRIX_SIDE, CV_8UC3,
        cv::Scalar(0, 0, 0)
) {
    // TODO: remove duplication!
    // border
    vector<cv::Point> borderPoints;
    for (const auto &borderPoint:borders) {
        borderPoints.emplace_back(borderPoint.x * POINT_DISCRETIZATION, borderPoint.y * POINT_DISCRETIZATION);
    }
    vector<vector<cv::Point>> container;
    container.push_back(borderPoints);
    cv::fillPoly(obstaclesShadow, container, cv::Scalar(255, 255, 255));


    // polygons
    vector<vector<cv::Point>> polygons;
    for (const auto &obstacle:obstacles) {
        vector<cv::Point> points;
        for (const auto &point:obstacle) {
            points.emplace_back(point.x * POINT_DISCRETIZATION, point.y * POINT_DISCRETIZATION);
        }
        polygons.push_back(points);
    }
    cv::fillPoly(obstaclesShadow, polygons, cv::Scalar(0, 0, 0));

    // gate
    vector<cv::Point> gatePoints;
    for (const auto &gatePoint:gate) {
        gatePoints.emplace_back(gatePoint.x * POINT_DISCRETIZATION, gatePoint.y * POINT_DISCRETIZATION);
    }
    vector<vector<cv::Point>> gateContainer;
    gateContainer.push_back(gatePoints);
    cv::fillPoly(obstaclesShadow, gateContainer, cv::Scalar(255, 255, 255));

    // victims
    vector<vector<cv::Point>> victimPolygons;
    for (const auto &victim:victims) {
        vector<cv::Point> points;
        for (const auto &point:victim) {
            points.emplace_back(point.x * POINT_DISCRETIZATION, point.y * POINT_DISCRETIZATION);
        }
        victimPolygons.push_back(points);
    }
    cv::fillPoly(obstaclesShadow, victimPolygons, cv::Scalar(255, 255, 255));

    showImageAndWait(obstaclesShadow, 0, "collision shadow");
}


bool ShadowCollisionDetector::doesCurveCollide(const DubinsCurve &curve) const {
    vector<Pose> poses = dubinsCurveToPoseVector(curve);
    for (auto pose:poses) {
        if (isPointInAnyObstacle(Point(pose.x, pose.y))) {
            return true;
        }
    }
    return false;
}


bool ShadowCollisionDetector::doesSegmentCollide(const Point &a, const Point &b) const {
    // TODO: Move discretization somewhere else
    double length = Graph::distanceBetween(a, b);
    double delta = 0.01;
    int numPoints = (length / delta);

    double xDiff = (b.x - a.x) / ((double) numPoints);
    double yDiff = (b.y - a.y) / ((double) numPoints);

    //DebugImage::drawPoint(a, cv::Scalar(200, 0, 0));
    //DebugImage::drawPoint(b, cv::Scalar(200, 0, 0));
    //DebugImage::showAndWait();

    for (int i = 0; i < numPoints; i++) {
        Point p(a.x + xDiff * (double)i, a.y + yDiff * (double)i);
        //DebugImage::drawPoint(p, cv::Scalar(100, 200, 100));
        //DebugImage::showAndWait();
        if(isPointInAnyObstacle(p)) {
            return true;
        }
    }
    return false;
}


inline bool ShadowCollisionDetector::isPointInAnyObstacle(const Point &point) const {
    int approxX = point.x * POINT_DISCRETIZATION;
    int approxY = point.y * POINT_DISCRETIZATION;
    if (approxX < 0 || approxY < 0 || approxX > OBSTACLES_MATRIX_SIDE || approxY > OBSTACLES_MATRIX_SIDE) {
        return true;
    }
    auto pixel = obstaclesShadow.at<cv::Vec3b>(cv::Point(approxX, approxY));
    return pixel[0] == 0;
}

