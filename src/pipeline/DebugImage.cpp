
#include "DebugImage.h"
#include "../opencv-utils.h"

using namespace student;


cv::Mat DebugImage::image(1000, 1280, CV_8UC3, cv::Scalar(0, 0, 255));


void student::DebugImage::showAndWait(int wait) {
    showImageAndWaitKeyPress(image, wait);
}


void DebugImage::drawSegment(const Point &a, const Point &b, int multiply) {
    cv::line(image, cv::Point(a.x * multiply, a.y * multiply), cv::Point(b.x * multiply, b.y * multiply),
             cv::Scalar(255, 0, 0));
}

void DebugImage::clear() {
    image.setTo(cv::Scalar(0, 0, 255));
}

void DebugImage::drawGraph(student::Graph graph) {
    for (const auto edge : graph.edges) {
        DebugImage::drawSegment(*edge.first, *edge.second, 800);// TODO: Move this constant away
    }
}

void DebugImage::drawImage(const cv::Mat &mat) {
    mat.copyTo(image);
}
