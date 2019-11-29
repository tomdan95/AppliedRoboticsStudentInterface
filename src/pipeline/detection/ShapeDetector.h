#ifndef STUDENT_PROECT_SHAPEDETECTOR_H
#define STUDENT_PROECT_SHAPEDETECTOR_H

#include "student_image_elab_interface.hpp"

using namespace std;

/**
 * 1. apply color mask
 * 2. find contours
 */
namespace  student {
    class ShapeDetector {

    protected:
        virtual cv::Mat applyColorMask(cv::Mat &hsvImage) = 0;

        vector<vector<cv::Point>> findContours(cv::Mat &filteredImage);

        vector<Polygon> findAllPolygons(vector<vector<cv::Point>> contours, double scale);

        virtual vector<Polygon> filterPolygons(vector<Polygon> polygons) {
            return polygons;
        }

        // TODO: map


    public:
        vector<Polygon> findPolygons(cv::Mat &hsvImage, double scale);
    };

}

#endif //STUDENT_PROECT_SHAPEDETECTOR_H
