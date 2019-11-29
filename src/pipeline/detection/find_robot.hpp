#ifndef STUDENT_PROECT_FIND_ROBOT_HPP
#define STUDENT_PROECT_FIND_ROBOT_HPP

#include "student_image_elab_interface.hpp"
#include "ShapeDetector.h"

using namespace std;

namespace student {

    class RobotPose {
    public:
        const Polygon polygon;
        const double x, y;
        const double theta;

        RobotPose(const Polygon &polygon, const double x, const double y, const double theta): polygon(polygon),
                                                                                               x(x), y(y),
                                                                                               theta(theta) {};
    };

    class RobotDetector : public ShapeDetector<RobotPose> {
    private:
        RobotPose getRobotPose(Polygon robot);

    protected:
        cv::Mat applyColorMask(cv::Mat &hsvImage) override;
        vector<Polygon> filterPolygons(vector<Polygon> polygons) override;
        vector<RobotPose> mapPolygons(vector<Polygon> polygons) override;

    public:
        RobotDetector() : ShapeDetector(10) {}
    };

}
#endif //STUDENT_PROECT_FIND_ROBOT_HPP
