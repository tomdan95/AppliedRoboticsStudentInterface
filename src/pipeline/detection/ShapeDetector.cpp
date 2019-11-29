#include "ShapeDetector.h"
#include "find_robot.hpp"
#include "find_victims.hpp"

using namespace std;


namespace student {

    template<class DetectedShape>
    vector<DetectedShape> ShapeDetector<DetectedShape>::findPolygons(cv::Mat &hsvImage, double scale) {
        cv::Mat filteredImage = applyColorMask(hsvImage);
        vector<vector<cv::Point>> contours = findContours(filteredImage);
        vector<Polygon> polygons = mapContoursToPolygon(contours, scale);
        vector<Polygon> filteredPolygons = filterPolygons(polygons);
        return mapPolygons(filteredPolygons);
    }

    template<class DetectedShape>
    vector<vector<cv::Point>> ShapeDetector<DetectedShape>::findContours(cv::Mat &filteredImage) {
        vector<vector<cv::Point>> contours;
        cv::findContours(filteredImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return contours;
    }

    template<class DetectedShape>
    vector<Polygon> ShapeDetector<DetectedShape>::mapContoursToPolygon(const vector<vector<cv::Point>> &contours, const double scale) {
        vector<Polygon> polygons;
        for (auto &contour : contours) {
            Polygon scaled_contour = mapContourToPolygon(contour, scale);
            polygons.push_back(scaled_contour);
        }
        return polygons;
    }

    template<class DetectedShape>
    Polygon ShapeDetector<DetectedShape>::mapContourToPolygon(const vector<cv::Point> &contour, const double scale) {
        vector<cv::Point> approxCurve;
        cv::approxPolyDP(contour, approxCurve, approxPolyEpsilon, true);
        Polygon scaled_contour;
        for (const auto &pt: approxCurve) {
            scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
        }
        return scaled_contour;
    }

    template<> vector<Polygon> ShapeDetector<Polygon>::mapPolygons(vector<Polygon> polygons) {
        return polygons;
    }

    /**
     * Due to compilation units, we need to list all the possible DetectedShape that we'll use inside the program.
     * Otherwise, the implementation won't be generated and when the linker won't be able to link the program.
     * TODO: This violates OCP, maybe better to move the definition of the template methods in the header file?
     */
    template class ShapeDetector<RobotPose>;
    template class ShapeDetector<Victim>;

}

