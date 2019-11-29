/**
 * To execute this function, set 'robot_detector' to false.
 */

#include "find_robot.hpp"
#include "utils.h"

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
        vector<Polygon> robots = detector.findPolygons(hsv, scale);
        if (robots.size() != 1) {
            return false;
        }
        computeRobotOrientationAndBaricenter(robots[0], x, y, theta);
        return true;
    }

    cv::Mat RobotDetector::applyColorMask(cv::Mat &hsvImage) {
        cv::Mat blueMask;
        cv::inRange(hsvImage, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), blueMask);
        return blueMask;
    }


    vector<Polygon> RobotDetector::filterPolygons(vector<Polygon> polygons) {
        vector<Polygon> filtered;
        for (auto &polygon : polygons) {
            if (polygon.size() == 3) {
                vector<cv::Point> points = getPointsFromPolygon(polygon);
                double area = cv::contourArea(points);
                if (area >= 300 && area <= 3000) {
                    filtered.push_back(polygon);
                }
            }
        }
        return filtered;
    }


    void computeRobotOrientationAndBaricenter(Polygon robot, double &x, double &y, double &theta) {
        double cx = 0, cy = 0;
        for (auto item: robot) {
            cx += item.x;
            cy += item.y;
        }
        cx /= robot.size();
        cy /= robot.size();

        double dst = 0;
        Point vertex;
        for (auto &item: robot) {
            double dx = item.x - cx;
            double dy = item.y - cy;
            double curr_d = dx * dx + dy * dy;
            if (curr_d > dst) {
                dst = curr_d;
                vertex = item;
            }
        }
        double dx = cx - vertex.x;
        double dy = cy - vertex.y;
        x = cx;
        y = cy;
        theta = std::atan2(dy, dx);
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