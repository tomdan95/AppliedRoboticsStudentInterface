#ifndef STUDENT_PROECT_FIND_ROBOT_HPP
#define STUDENT_PROECT_FIND_ROBOT_HPP

#include "student_image_elab_interface.hpp"
#include "ShapeDetector.h"

using namespace std;

namespace student {


    class RobotDetector : public ShapeDetector {
    private:
        cv::Mat applyColorMask(cv::Mat &hsvImage) override;

        vector<Polygon> filterPolygons(vector<Polygon> polygons) override;
    };

    void computeRobotOrientationAndBaricenter(Polygon robot, double &x, double &y, double &theta);

}
#endif //STUDENT_PROECT_FIND_ROBOT_HPP
