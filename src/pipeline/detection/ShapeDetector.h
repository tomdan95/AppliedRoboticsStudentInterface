#ifndef STUDENT_PROECT_SHAPEDETECTOR_H
#define STUDENT_PROECT_SHAPEDETECTOR_H

#include <utility>

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

    protected:
        virtual cv::Mat applyColorMask(const cv::Mat &hsvImage) = 0;

        vector<vector<cv::Point>> findContours(const cv::Mat &filteredImage);

        // TODO: implement filterContours and make virtual filterContour, so we don'thave to cpy and paste the loop logic
        virtual vector<vector<cv::Point>> filterContours(const vector<vector<cv::Point>> &contours);

        vector<Polygon> mapContoursToPolygon(const vector<vector<cv::Point>> &contours, double scale);

        Polygon mapContourToPolygon(const vector<cv::Point> &contour, double scale);

        // TODO: implement mapPolygons and make virtual makePolygon, so we don'thave to cpy and paste the loop logic
        virtual vector<DetectedShape>
        mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contour, const cv::Mat &hsvImage,
                    const cv::Mat &filteredImage) = 0;

    public:
        explicit ShapeDetector(const int approxPolyEpsilon) : approxPolyEpsilon(approxPolyEpsilon) {};

        vector<DetectedShape> findPolygons(const cv::Mat &hsvImage, double scale);
    };
}

#endif //STUDENT_PROECT_SHAPEDETECTOR_H
