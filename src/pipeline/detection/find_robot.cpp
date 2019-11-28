/**
 * To execute this function, set 'robot_detector' to false.
 */

#include "find_robot.hpp"

using namespace std;

namespace student {
//-------------------------------------------------------------------------
    //          FIND ROBOT
    //-------------------------------------------------------------------------
    bool processRobot(const cv::Mat& hsv_img, const double scale, Polygon& triangle, double& x, double& y, double& theta){

        cv::Mat blue_mask;

        cv::inRange(hsv_img, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), blue_mask);

        // Process blue mask
        std::vector<std::vector<cv::Point>> contours, contours_approx;
        std::vector<cv::Point> approx_curve;
        cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // cv::imshow("filterrrr", blue_mask);
        // cv::waitKey(1);

        // cv::Mat contours_img;
        // contours_img = hsv_img.clone();

        // drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
        // std::cout << "N. contours: " << contours.size() << std::endl;


        bool found = false;
        for (int i=0; i<contours.size(); ++i)
        {
            //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;

            cv::approxPolyDP(contours[i], approx_curve, 10, true);
            contours_approx = {approx_curve};

            // cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

            double area = cv::contourArea(approx_curve);

            if (approx_curve.size() != 3) continue;

            if (area < 300 || area>3000) continue;


            found = true;
            break;
        }

        if (found)
        {
            for (const auto& pt: approx_curve) {
                triangle.emplace_back(pt.x/scale, pt.y/scale);
            }

            double cx = 0, cy = 0;
            for (auto item: triangle)
            {
                cx += item.x;
                cy += item.y;
            }
            cx /= triangle.size();
            cy /= triangle.size();

            double dst = 0;
            Point vertex;
            for (auto& item: triangle)
            {
                double dx = item.x-cx;
                double dy = item.y-cy;
                double curr_d = dx*dx + dy*dy;
                if (curr_d > dst)
                {
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

            double dx = cx-vertex.x;
            double dy = cy-vertex.y;

            x = cx;
            y = cy;
            theta = std::atan2(dy, dx);


            //covariance = {};

            //std::cout << xc_m << " " << yc_m << " " << theta*180/M_PI << std::endl;
        }

        // cv::imshow("Original", contours_img);
        // cv::waitKey(1);

        return found;
    }

    /*!
     * Process the image to detect the robot position
     * @param[in]  image_in       input image
     * @param[in]  scale          1px/scale = X meters
     * @param[out] triangle       polygon defined from triangle corners
     * @param[out] x              x position of the robot (i.e. baricenter of the triangle) in arena reference system
     * @param[out] y              y position of the robot (i.e. baricenter of the triangle) in arena reference system
     * @param[out] theta          yaw of the robot in the arena reference system
     * @param[in]  config_folder  A custom string from config file.
    */
    bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){

        // Convert color space from BGR to HSV
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
        cout << "process robot" << endl;
        return processRobot(hsv_img, scale, triangle, x, y, theta);
    }


}