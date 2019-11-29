

#include "utils.h"

using namespace std;
namespace student {
    cv::Mat convertRGBToHSV(const cv::Mat &rgb) {
        cv::Mat hsv;
        cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
        return hsv;
    }

// TODO: Do we really need this?
    vector<cv::Point> getPointsFromPolygon(Polygon &polygon) {
        vector<cv::Point> points;
        for (auto &p : polygon) {
            points.emplace_back(p.x, p.y);
        }
        return points;
    }

}