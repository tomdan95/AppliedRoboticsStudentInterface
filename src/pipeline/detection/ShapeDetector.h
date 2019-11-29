#ifndef STUDENT_PROECT_SHAPEDETECTOR_H
#define STUDENT_PROECT_SHAPEDETECTOR_H

#include "student_image_elab_interface.hpp"

using namespace std;

/**
 * 1. apply color mask
 * 2. find contours
 */
namespace student {

    template<class DetectedShape>
    class ShapeDetector {
    private:
        const int approxPolyEpsilon;

        vector<vector<cv::Point>> findContours(cv::Mat &filteredImage);
        vector<Polygon> mapContoursToPolygon(const vector<vector<cv::Point>> &contours, double scale);
        Polygon mapContourToPolygon(const vector<cv::Point> &contour, double scale);

    protected:
        virtual cv::Mat applyColorMask(cv::Mat &hsvImage) = 0;
        virtual vector<Polygon> filterPolygons(vector<Polygon> polygons) {
            return polygons;
        }

        virtual vector<DetectedShape> mapPolygons(vector<Polygon> polygons) = 0;

    public:
        explicit ShapeDetector(const int approxPolyEpsilon) : approxPolyEpsilon(approxPolyEpsilon) {};

        vector<DetectedShape> findPolygons(cv::Mat &hsvImage, double scale);
    };
}

#endif //STUDENT_PROECT_SHAPEDETECTOR_H
