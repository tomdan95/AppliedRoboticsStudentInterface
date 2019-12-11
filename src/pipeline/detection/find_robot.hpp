#ifndef STUDENT_PROECT_FIND_ROBOT_HPP
#define STUDENT_PROECT_FIND_ROBOT_HPP

#include <utility>

#include "student_image_elab_interface.hpp"
#include "ShapeDetector.h"

using namespace std;

namespace student {

    class RobotPose {
    public:
        const Polygon polygon;
        const double x, y;
        const double theta;

        RobotPose(Polygon polygon, const double x, const double y, const double theta) : polygon(std::move(polygon)),
                                                                                         x(x), y(y),
                                                                                         theta(theta) {};
    };

    class RobotDetector : public ShapeDetector<RobotPose> {
    private:
        RobotPose getRobotPose(const Polygon &robot);

    protected:
        cv::Mat applyColorMask(const cv::Mat &hsvImage) override;

        vector<vector<cv::Point>> filterContours(const vector<vector<cv::Point>> &contours) override;

        vector<RobotPose>
        mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contour, const cv::Mat &hsvImage,
                    const cv::Mat &filteredImage) override;

    public:
        RobotDetector() : ShapeDetector(10) {}

        double getRobotTheta(const Polygon &robot, const Point &center) const;
    };

}
#endif //STUDENT_PROECT_FIND_ROBOT_HPP
