/**
 * To execute this function, set 'robot_detector' to false.
 */

#include "find_robot.hpp"

using namespace std;

namespace student {

    cv::Mat convertRGBToHSV(const cv::Mat &rgb);

    bool
    processRobot(const cv::Mat &hsvImage, const double scale, Polygon &triangle, double &x, double &y, double &theta);

    cv::Mat getBlueMask(const cv::Mat &hsv);


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
        return processRobot(convertRGBToHSV(rgbImage), scale, triangle, x, y, theta);
    }

    cv::Mat convertRGBToHSV(const cv::Mat &rgb) {
        cv::Mat hsv;
        cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
        return hsv;
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

    bool
    processRobot(const cv::Mat &hsvImage, const double scale, Polygon &triangle, double &x, double &y, double &theta) {
        cv::Mat blue_mask = getBlueMask(hsvImage);

        // Process blue mask
        std::vector<std::vector<cv::Point>> contours, contours_approx;
        std::vector<cv::Point> approx_curve;
        cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // TODO: make debugContoursImage as private attribute
        cv::Mat debugContoursImage = debugShowBlueFilterAndContours(hsvImage, blue_mask, contours);

        bool found = findRobotTriangle(contours, contours_approx, approx_curve, debugContoursImage);
        if (!found) {
            return false;
        }

        // TODO: A side effect of findRobotTriangle is that it changes approx_curve to the curve of the triangle.
        // TODO: make it clearer

        for (const auto &pt: approx_curve) {
            triangle.emplace_back(pt.x / scale, pt.y / scale);
        }

        // TODO: Extract code to find baricenter into a function
        double cx = 0, cy = 0;
        for (auto item: triangle) {
            cx += item.x;
            cy += item.y;
        }
        cx /= triangle.size();
        cy /= triangle.size();

        double dst = 0;
        Point vertex;
        for (auto &item: triangle) {
            double dx = item.x - cx;
            double dy = item.y - cy;
            double curr_d = dx * dx + dy * dy;
            if (curr_d > dst) {
                dst = curr_d;
                vertex = item;
            }
        }

        // cv::Moments m = cv::moments(approx_curve, false);
        // cv::Point center(m.m10/m.m00, m.m01/m.m00);
        // cv::Vec4f line;
        // cv::fitLine(approx_curve, line, cv::DIST_L2, 0, 0.01, 0.01);
        // cv::line(warpedFrame, cv::Point(line[2], line[3]), cv::Point(line[2]+line(0)*80, line(3)+line(1)*80), (0,255,0), 2);


        //cv::line(contours_img, center*scale, vertex*scale, (0,255,0), 2);
        //cv::circle(contours_img, center*scale, 20, cv::Scalar(0,0,0), -1);

        double dx = cx - vertex.x;
        double dy = cy - vertex.y;

        x = cx;
        y = cy;
        theta = std::atan2(dy, dx);


        //covariance = {};

        //std::cout << xc_m << " " << yc_m << " " << theta*180/M_PI << std::endl;


        cv::imshow("Original", debugContoursImage);
        cv::waitKey(0);

        return true;
    }

    cv::Mat getBlueMask(const cv::Mat &hsv) {
        cv::Mat blueMask;
        //cv::inRange(hsv, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), blueMask);
        cv::inRange(hsv, cv::Scalar(115, 90, 60), cv::Scalar(130, 150, 255), blueMask);

        return blueMask;
    }

    bool findRobotTriangle(vector<std::vector<cv::Point>> &contours, vector<vector<cv::Point>> &contoursApprox,
                           vector<cv::Point> &approxCurve, cv::Mat &contoursImg) {
        for (auto &contour : contours) {
            cv::approxPolyDP(contour, approxCurve, 10, true);
            contoursApprox = {approxCurve};

            cv::drawContours(contoursImg, contoursApprox, -1, cv::Scalar(0, 170, 220), 3, cv::LINE_AA);

            if (approxCurve.size() == 3) {
                double area = cv::contourArea(approxCurve);
                if (area >= 300 && area <= 3000) {
                    return true;
                }
            }
        }
        return false;
    }

}