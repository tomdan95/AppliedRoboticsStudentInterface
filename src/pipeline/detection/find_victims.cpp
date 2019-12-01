#include "find_victims.hpp"


namespace student {
    cv::Mat VictimDetector::applyColorMask(const cv::Mat &hsvImage) {
        cv::Mat greenMask;
        cv::inRange(hsvImage, cv::Scalar(45, 50, 50), cv::Scalar(75, 255, 255), greenMask);
        return greenMask;
    }

    vector<Victim> student::VictimDetector::mapPolygons(vector<Polygon> polygons) {
        vector<Victim> victims;
        for (auto &polygon : polygons) {
            victims.emplace_back(polygon, -1);
        }
        return victims;
    }
}
