#include "CollisionDetector.h"
#include "../planning.h"
#include "../../../opencv-utils.h"

using namespace std;
using namespace student;

// TODO: Use black and white image to save space
CollisionDetector::CollisionDetector(vector<Polygon> obstacles) : obstaclesShadow(OBSTACLES_MATRIX_SIDE,
                                                                                  OBSTACLES_MATRIX_SIDE, CV_8UC3,
                                                                                  cv::Scalar(255, 255, 255)) {
    vector<vector<cv::Point>> polygons;
    for (auto obstacle:obstacles) {
        vector<cv::Point> points;
        for (auto point:obstacle) {
            points.emplace_back(point.x, point.y);
        }
        polygons.push_back(points);
    }
    cv::fillPoly(obstaclesShadow, polygons, cv::Scalar(0, 0, 0));
    //showImageAndWaitKeyPress(obstaclesShadow);
}


bool CollisionDetector::isPointInAnyObstacle(const Point &point, vector<Polygon> obstacles) {
    for (const auto &obstacle:obstacles) {
        if (isPointInObstacle(point, obstacle)) {
            return true;
        }
    }
    return false;
}


int CollisionDetector::isPointInObstacle(const Point p, const Polygon &polygon) {
    int crossingCount = 0;

    Polygon copy = polygon;
    copy.push_back(polygon[0]);

    for (int i = 0; i < polygon.size(); i++) {
        Point a = copy[i];
        Point b = copy[i + 1];

        if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            crossingCount++;
        }
    }
    return (crossingCount % 2 != 0);

}

bool CollisionDetector::doesCurveCollide(DubinsCurve curve) {
    vector<Pose> poses = dubinsCurveToPoseVector(curve);
    for (auto pose:poses) {
        if (isPointInAnyObstacle(Point(pose.x, pose.y))) {
            return true;
        }
    }
    return false;
}

bool CollisionDetector::isPointInAnyObstacle(const Point &point) {
    int approxX = point.x;
    int approxY = point.y;
    if(approxX < 0 || approxY < 0) {
        return true;
    }
    auto pixel = obstaclesShadow.at<cv::Vec3b>(cv::Point(approxX, approxY));
    return pixel[0] == 0;
}

