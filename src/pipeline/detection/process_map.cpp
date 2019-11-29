/**
 * To execute this function, set 'obstacle_detector' to false.
 */


#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"


namespace student {
    bool processObstacles(const cv::Mat &hsv_img, const double scale, std::vector<Polygon> &obstacle_list) {

        // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
        cv::Mat red_mask_low, red_mask_high, red_mask;
        cv::inRange(hsv_img, cv::Scalar(0, 10, 10), cv::Scalar(15, 255, 255), red_mask_low);
        cv::inRange(hsv_img, cv::Scalar(175, 10, 10), cv::Scalar(179, 255, 255), red_mask_high);
        cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);

        // cv::Mat img_small;
        // cv::resize(red_mask, img_small, cv::Size(640, 512));


        std::vector<std::vector<cv::Point>> contours, contours_approx;
        std::vector<cv::Point> approx_curve;
        cv::Mat contours_img;

        // Process red mask
        //contours_img = img_in.clone();
        cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
        //std::cout << "N. contours: " << contours.size() << std::endl;
        for (int i = 0; i < contours.size(); ++i) {
            //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
            approxPolyDP(contours[i], approx_curve, 3, true);

            Polygon scaled_contour;
            for (const auto &pt: approx_curve) {
                scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
            }
            obstacle_list.push_back(scaled_contour);
            //contours_approx = {approx_curve};
            //drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
            //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
        }
        //std::cout << std::endl;
        // cv::imshow("Original", contours_img);
        // cv::waitKey(1);

        return true;
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

    bool processVictims(const cv::Mat &hsv_img, const double scale, std::vector<std::pair<int, Polygon>> &victim_list) {

        // Find green regions
        cv::Mat green_mask;
        cv::inRange(hsv_img, cv::Scalar(45, 50, 50), cv::Scalar(75, 255, 255), green_mask);


        std::vector<std::vector<cv::Point>> contours, contours_approx;
        std::vector<cv::Point> approx_curve;
        //cv::Mat contours_img;

        // Process red mask
        //contours_img = hsv_img.clone();
        cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
        //std::cout << "N. contours: " << contours.size() << std::endl;
        for (int i = 0; i < contours.size(); ++i) {
            //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
            approxPolyDP(contours[i], approx_curve, 1, true);

            Polygon scaled_contour;
            for (const auto &pt: approx_curve) {
                scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
            }
            victim_list.push_back({i + 1, scaled_contour});
            //contours_approx = {approx_curve};
            //drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
            //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
        }


        // cv::imshow("Original", contours_img);
        // cv::waitKey(1);

        return true;
    }

    bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list,
                    std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate,
                    const std::string &config_folder) {
        // Convert color space from BGR to HSV
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);


        const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
        if (!res1) std::cout << "processObstacles return false" << std::endl;
//        const bool res2 = processGate(hsv_img, scale, gate);
//        if(!res2) std::cout << "processGate return false" << std::endl;
        const bool res3 = processVictims(hsv_img, scale, victim_list);
        if (!res3) std::cout << "processVictims return false" << std::endl;

        return true;
        //return res1 /*&& res2*/ && res3;
    }

}