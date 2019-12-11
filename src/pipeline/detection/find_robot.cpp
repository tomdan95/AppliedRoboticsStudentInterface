/**
 * To execute this function, set 'robot_detector' to false.
 */

#include "find_robot.hpp"
#include "../../opencv-utils.h"
#include "../utils.h"

using namespace std;


namespace student {

    /*!
     * Process the image to detect the robot position
     * @param[in]  rgbImage       input image
     * @param[in]  scale          1px/scale = X meters
     * @param[out] triangle       polygon defined from triangle corners
     * @param[out] x              x position of the robot (i.e. baricenter of the triangle) in arena reference system
     * @param[out] y              y position of the robot (i.e. baricenter of the triangle) in arena reference system
     * @param[out] theta          yaw of the robot in the arena reference system
     * @param[in]  config_folder  A custom string from config file.
    */
    bool findRobot(const cv::Mat &rgbImage, const double scale, Polygon &triangle, double &x, double &y, double &theta,
                   const std::string &config_folder) {
        cv::Mat hsv = convertRGBToHSV(rgbImage);
        RobotDetector detector;
        vector<RobotPose> robots = detector.findPolygons(hsv, scale);
        if (robots.size() != 1) {
            return false;
        }
        RobotPose robot = robots[0];
        x = robot.x;
        y = robot.y;
        theta = robot.theta;
        triangle = robot.polygon;
        return true;
    }

    cv::Mat RobotDetector::applyColorMask(const cv::Mat &hsvImage) {
        cv::Mat blueMask;
        cv::inRange(hsvImage, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), blueMask);
        return blueMask;
    }


    vector<vector<cv::Point>> RobotDetector::filterContours(const vector<vector<cv::Point>> &contours) {
        vector<vector<cv::Point>> filtered;
        for (auto &contour : contours) {
            if (contour.size() == 3) {
                double area = cv::contourArea(contour);
                if (area >= 300 && area <= 3000) {
                    filtered.push_back(contour);
                }
            }
        }
        return filtered;
    }

    vector<RobotPose> RobotDetector::mapPolygons(vector<Polygon> polygons, const vector<vector<cv::Point>> &contour,
                                                 const cv::Mat &hsvImage, const cv::Mat &filteredImage) {
        vector<RobotPose> poses;
        for (auto &polygon : polygons) {
            poses.push_back(getRobotPose(polygon));
        }
        return poses;
    }


    RobotPose RobotDetector::getRobotPose(const Polygon &robot) {
        Point center = getPolygonCenter(robot);
        RobotPose pose(robot, center.x, center.y, getRobotTheta(robot, center));
        return pose;
    }

    double RobotDetector::getRobotTheta(const Polygon &robot, const Point &center) const {
        double dst = 0;
        const Point *vertex;
        for (auto &item: robot) {
            double dx = item.x - center.x;
            double dy = item.y - center.y;
            double curr_d = dx * dx + dy * dy;
            if (curr_d > dst) {
                dst = curr_d;
                vertex = &item;
            }
        }
        double dx = center.x - vertex->x;
        double dy = center.y - vertex->y;
        double theta = atan2(dy, dx);
        return theta;
    }


    cv::Mat debugShowBlueFilterAndContours(const cv::Mat &hsvImage, const cv::Mat &blue_mask,
                                           const vector<std::vector<cv::Point>> &contours) {
        cv::imshow("filter", blue_mask);
        cout << "[DEBUG] Now showing the image after the blue filter" << endl;
        cout << "[DEBUG] Press a key to show the found contours" << endl;
        cv::waitKey(0);

        cv::Mat contours_img;
        contours_img = hsvImage.clone();

        drawContours(contours_img, contours, -1, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
        cout << "[DEBUG] Showing the " << contours.size() << " found contours." << endl;
        cout << "[DEBUG] Press a key to continue" << endl;
        cv::waitKey(0);
        return contours_img;
    }
}