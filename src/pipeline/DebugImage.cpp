
#include "DebugImage.h"
#include "../opencv-utils.h"

#define DEFAULT_MULTIPLY 400

using namespace student;


cv::Mat DebugImage::image(1000, 1280, CV_8UC3, cv::Scalar(0, 0, 255));


void student::DebugImage::showAndWait(int wait) {
    showImageAndWait(image, wait);
}


void DebugImage::drawSegment(const Point &a, const Point &b, int multiply, cv::Scalar color) {
    cv::line(image, cv::Point(a.x * multiply, a.y * multiply), cv::Point(b.x * multiply, b.y * multiply),
             color);
}

void DebugImage::clear() {
    image.setTo(cv::Scalar(0, 0, 255));
}

void DebugImage::drawGraph(student::Graph graph) {
    for (const auto &pointWithAdjacents : graph.edges) {
        for (const auto &adjacentPoint : pointWithAdjacents.second) {
            DebugImage::drawSegment(*pointWithAdjacents.first, *adjacentPoint, DEFAULT_MULTIPLY);// TODO: Move this constant away
        }
    }
}

void DebugImage::drawImage(const cv::Mat &mat) {
    mat.copyTo(image);
}

void DebugImage::drawPath(vector<Point *> path, cv::Scalar color) {
    Point *start = path[0];
    drawPoint(*start, color);
    for (int i = 1; i < path.size(); i++) {
        drawSegment(*start, *path[i], DEFAULT_MULTIPLY, color);
        drawPoint(*path[i], color);
        start = path[i];
    }
}

void DebugImage::drawPoint(Point point, cv::Scalar color) {
    cv::circle(image, cv::Point(point.x * DEFAULT_MULTIPLY, point.y * DEFAULT_MULTIPLY), 5, color, 3);
}

void DebugImage::drawPoses(vector<Pose> poses) {
    Pose start = poses[0];
    drawPose(start);
    for (int i = 1; i < poses.size(); i++) {
        Pose end = poses[i];
        drawSegment(Point(start.x, start.y), Point(end.x, end.y), DEFAULT_MULTIPLY, cv::Scalar(122, 122, 0));
        drawPose(end);
        start = end;
    }
}

void DebugImage::drawPose(Pose pose) {
    // TODO: Draw triangle
    drawPoint(Point(pose.x, pose.y), cv::Scalar(200, 0, 200));
}

void DebugImage::drawPolygons(const vector<Polygon> &polygons, int multiply, cv::Scalar color) {
    for (const auto &polygon:polygons) {
        drawPolygon(polygon, multiply, color);
    }
}

void DebugImage::drawPolygon(const Polygon &polygon, int multiply, cv::Scalar color) {
    Point start(polygon[0].x, polygon[0].y);
    for (int i = 1; i < polygon.size(); i++) {
        Point end(polygon[i].x, polygon[i].y);
        drawSegment(start, end, multiply, color);
        start = end;
    }
    Point end(polygon[0].x, polygon[0].y);
    drawSegment(start, end, multiply, color);

}