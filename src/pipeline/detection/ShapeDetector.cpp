#include "ShapeDetector.h"
#include "find_robot.hpp"
#include "find_victims.hpp"

using namespace std;


namespace student {

    template<class DetectedShape>
    vector<DetectedShape> ShapeDetector<DetectedShape>::findPolygons(const cv::Mat &hsvImage, double scale) {
        
        cv::Mat filteredImage = applyColorMask(hsvImage);
        vector<vector<cv::Point>> contours = findContours(filteredImage);
        vector<vector<cv::Point>> filteredContours = filterContours(contours);
        vector<Polygon> polygons = mapContoursToPolygon(filteredContours, scale);
        return mapPolygons(polygons, filteredContours, hsvImage, filteredImage);
    }

    template<class DetectedShape>
    vector<vector<cv::Point>> ShapeDetector<DetectedShape>::findContours(const cv::Mat &filteredImage) {
        vector<vector<cv::Point>> contours;
        cv::findContours(filteredImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        vector<vector<cv::Point>> approximatedContours;

        for (vector<cv::Point> contour : contours) {
            vector<cv::Point> approximatedContour;
            cv::approxPolyDP(contour, approximatedContour, approxPolyEpsilon, true);
            approximatedContours.push_back(approximatedContour);
        }

        return approximatedContours;
    }

    template<class DetectedShape>
    vector<Polygon> ShapeDetector<DetectedShape>::mapContoursToPolygon(const vector<vector<cv::Point>> &contours, const double scale) {
        vector<Polygon> polygons;
        for (auto &contour : contours) {
            Polygon polygon = mapContourToPolygon(contour, scale);
            polygons.emplace_back(polygon);
        }
        return polygons;
    }

    template<class DetectedShape>
    Polygon ShapeDetector<DetectedShape>::mapContourToPolygon(const vector<cv::Point> &contour, const double scale) {
        vector<Point> polygonPoints;
        for (const auto &pt: contour) {
            polygonPoints.emplace_back(pt.x / scale, pt.y / scale);
        }
        return Polygon(polygonPoints);
    }

    template<class DetectedShape>
    vector<vector<cv::Point>> ShapeDetector<DetectedShape>::filterContours(const vector<vector<cv::Point>> &contours) {
        return contours;
    }

    /**
     * Due to compilation units, we need to list all the possible DetectedShape that we'll use inside the program.
     * Otherwise, the implementation won't be generated and when the linker won't be able to link the program.
     * TODO: This violates OCP, maybe better to move the definition of the template methods in the header file?
     */
    template class ShapeDetector<RobotPose>;
    template class ShapeDetector<Victim>;
    template class ShapeDetector<Polygon>;
}

