/**
 * To execute this function, set 'obstacle_detector' to false.
 */


#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "../../utils.h"
#include "find_victims.hpp"
#include "find_obstacles.hpp"


namespace student {
    bool processObstacles(const cv::Mat &hsv_img, const double scale, std::vector<Polygon> &obstacle_list) {
        ObstacleDetector detector;
        vector<Polygon> obstacles = detector.findPolygons(hsv_img, scale);
        cout << "[PROCESS_MAP] Found " << obstacles.size() << " obstacles" << endl;
        if (obstacles.empty()) {
            return false;
        } else {
            for(const auto &obstacle : obstacles) {
                obstacle_list.push_back(obstacle);
            }
            return true;
        }
    }


    bool processGate(const cv::Mat &hsv_img, const double scale, Polygon &gate) {

        // Find purple regions
        cv::Mat purple_mask;
        cv::inRange(hsv_img, cv::Scalar(130, 10, 10), cv::Scalar(165, 255, 255), purple_mask);


        std::vector<std::vector<cv::Point>> contours, contours_approx;
        std::vector<cv::Point> approx_curve;
        //cv::Mat contours_img;

        // Process purple mask
        //contours_img = hsv_img.clone();
        cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
        // std::cout << "N. contours: " << contours.size() << std::endl;


        bool res = false;

        for (auto &contour : contours) {
            const double area = cv::contourArea(contour);
            //std::cout << "AREA " << area << std::endl;
            //std::cout << "SIZE: " << contours.size() << std::endl;
            if (area > 500) {
                approxPolyDP(contour, approx_curve, 3, true);

                // contours_approx = {approx_curve};
                // drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);


                for (const auto &pt: approx_curve) {
                    gate.emplace_back(pt.x / scale, pt.y / scale);
                }
                res = true;
                break;
            }
        }


        // cv::imshow("Original", contours_img);
        // cv::waitKey(1);

        return true;
        //return res;
    }

    bool processVictims(const cv::Mat &hsvImage, const double scale, std::vector<std::pair<int, Polygon>> &victim_list) {
        VictimDetector detector;
        vector<Victim> victims = detector.findPolygons(hsvImage, scale);
        cout << "[PROCESS_MAP] Found " << victims.size() << " victims" << endl;
        if (victims.empty()) {
            return false;
        } else {
            for(const auto &victim : victims) {
                victim_list.emplace_back(victim.number, victim.polygon);
            }
            return true;
        }
    }

    bool processMap(const cv::Mat &rgbImage, const double scale, vector<Polygon> &obstacleList,
                    vector<pair<int, Polygon>> &victimList, Polygon &gate, const string &config_folder) {
        cv::Mat hsv_img = convertRGBToHSV(rgbImage);
        bool foundObstacles = processObstacles(hsv_img, scale, obstacleList);
        bool foundVictims = processVictims(hsv_img, scale, victimList);
        bool foundGate = processGate(hsv_img, scale, gate);
        return foundObstacles && foundVictims && foundGate;
    }

}