
#include "DebugImage.h"
#include "../opencv-utils.h"

using namespace student;


cv::Mat DebugImage::image(1000, 1280, CV_8UC3, cv::Scalar(0, 0, 255));


void student::DebugImage::showAndWait(int wait) {
    showImageAndWaitKeyPress(image, wait);
}


void DebugImage::drawSegment(const Point &a, const Point &b, int multiply, cv::Scalar color) {
    cv::line(image, cv::Point(a.x * multiply, a.y * multiply), cv::Point(b.x * multiply, b.y * multiply),
             color);
}

void DebugImage::clear() {
    image.setTo(cv::Scalar(0, 0, 255));
}

void DebugImage::drawGraph(student::Graph graph) {
    for (const auto& pointWithAdjacents : graph.edges) {
        for(const auto& adjacentPoint : pointWithAdjacents.second) {
            DebugImage::drawSegment(*pointWithAdjacents.first, *adjacentPoint, 800.0);// TODO: Move this constant away
        }
    }
}

void DebugImage::drawImage(const cv::Mat &mat) {
    mat.copyTo(image);
}

void DebugImage::drawPath(vector<Point *> path, cv::Scalar color) {
    Point *start = path[0];
    for (int i = 1; i < path.size(); i++) {
        drawSegment(*start, *path[i], 800, color);
        start = path[i];
    }
}

void DebugImage::drawPoint(Point point, cv::Scalar color) {

    cv::circle(image, cv::Point(point.x * 800.0, point.y * 800.0),5, color, 3);
}

void DebugImage::drawPoses(vector<Pose> poses) {
    Pose start = poses[0];
    drawPose(start);
    for (int i = 1; i < poses.size(); i++) {
        Pose end = poses[i];
        drawSegment(Point(start.x, start.y), Point(end.x, end.y), 800, cv::Scalar(122, 122, 0));
        drawPose(end);
        start = end;
    }
    Pose end = poses[0];
    drawSegment(Point(start.x, start.y), Point(end.x, end.y), 800, cv::Scalar(122, 122, 0));
}

void DebugImage::drawPose(Pose pose) {
    // TODO: Draw triangle
    drawPoint(Point(pose.x, pose.y), cv::Scalar(200, 0, 200));
}
