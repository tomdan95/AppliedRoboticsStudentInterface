#include "ShapeDetector.h"

using namespace std;


namespace student {
    vector<Polygon> ShapeDetector::findPolygons(cv::Mat &hsvImage, const double scale) {
        cv::Mat filteredImage = applyColorMask(hsvImage);
        vector<vector<cv::Point>> contours = findContours(filteredImage);
        vector<Polygon> polygons = findAllPolygons(contours, scale);
        return filterPolygons(polygons);
    }

    vector<vector<cv::Point>> ShapeDetector::findContours(cv::Mat &filteredImage) {
        vector<vector<cv::Point>> contours;
        cv::findContours(filteredImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }

    vector<Polygon> ShapeDetector::findAllPolygons(vector<vector<cv::Point>> contours, const double scale) {
        vector<cv::Point> approxCurve;
        vector<Polygon> polygons;
        for (int i = 0; i < contours.size(); i++) {
            //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
            //approxPolyDP(contours[i], approxCurve, 3, true);
            // TODO: Allow implementation classes to customize this
            cv::approxPolyDP(contours[i], approxCurve, 10, true);

            Polygon scaled_contour;
            for (const auto &pt: approxCurve) {
                scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
            }
            polygons.push_back(scaled_contour);
        }
        return polygons;
    }
}