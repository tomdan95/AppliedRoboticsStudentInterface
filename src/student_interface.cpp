#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "dubins/temp.h"
#include "dubins/dubins.h"
#include "dubins/curve.h"

#include <stdexcept>
#include <sstream>

namespace student {

    void loadImage(cv::Mat &img_out, const std::string &config_folder) {
        static bool initialized = false;
        static std::vector<cv::String> img_list; // list of images to load
        static size_t idx = 0;  // idx of the current img
        static size_t function_call_counter = 0;  // idx of the current img
        const static size_t freeze_img_n_step = 30; // hold the current image for n iteration
        static cv::Mat current_img; // store the image for a period, avoid to load it from file every time

        if (!initialized) {
            const bool recursive = false;
            // Load the list of jpg image contained in the config_folder/img_to_load/
            cv::glob(config_folder + "/img_to_load/*.jpg", img_list, recursive);

            if (img_list.size() > 0) {
                initialized = true;
                idx = 0;
                current_img = cv::imread(img_list[idx]);
                function_call_counter = 0;
            } else {
                initialized = false;
            }
        }

        if (!initialized) {
            throw std::logic_error("Load Image can not find any jpg image in: " + config_folder + "/img_to_load/");
            return;
        }

        img_out = current_img;
        function_call_counter++;

        // If the function is called more than N times load increment image idx
        if (function_call_counter > freeze_img_n_step) {
            function_call_counter = 0;
            idx = (idx + 1) % img_list.size();
            current_img = cv::imread(img_list[idx]);
        }
    }

    int imageCounter = 0;

    void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder) {


        cv::imshow(topic, img_in);

        std::cout << "showing image" << std::endl;

        char pressedKey = cv::waitKey(30);

        if (pressedKey == 's') {
            std::string fileName =
                    "/home/lar2019/AppliedRoboticsStudentInterface/calibrationImages/" + std::to_string(imageCounter) +
                    ".png";
            cv::imwrite(fileName, img_in);
            std::cout << "written image " << fileName << std::endl;
            imageCounter++;
        }

        //throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
    }

    bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                        cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder) {
        throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED");
    }

    void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                        const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder) {

        throw std::logic_error("STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED");

    }

    //-------------------------------------------------------------------------
    //          FIND PLANE TRANSFORM
    //-------------------------------------------------------------------------
    void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec,
                            const cv::Mat &tvec,
                            const std::vector<cv::Point3f> &object_points_plane,
                            const std::vector<cv::Point2f> &dest_image_points_plane,
                            cv::Mat &plane_transf, const std::string &config_folder) {

        cv::Mat image_points;

        // project points
        cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

        plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
    }

    //-------------------------------------------------------------------------
    //          UNWARP TRANSFORM
    //-------------------------------------------------------------------------
    void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf,
                const std::string &config_folder) {
        cv::warpPerspective(img_in, img_out, transf, img_in.size());
    }

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


    bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta,
                   const std::string &config_folder) {

        std::cout << "findRobot" << std::endl;

        // Convert color space from BGR to HSV
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img,
                     cv::COLOR_BGR2HSV);

        // Extract blue color region
        cv::Mat blue_mask;
        cv::inRange(hsv_img, cv::Scalar(100, 120, 150),
                    cv::Scalar(135, 255, 255), blue_mask);

        // BLUE MASK on real world imgs contains noise
        // to get rid of the noise a possible solution is to use:
        // cv::erode and cv::dilate (https://docs.opencv.org/3.3.1/db/df6/tutorial_erosion_dilatation.html)

        // Find blue mask contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blue_mask, contours,
                         cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

#ifdef FIND_ROBOT_DEBUG_PLOT // do this only if FIND_DEBUG_PLOT is defined
        cv::imshow("findRobotHsv", hsv_img);
        cv::imshow("findRobotMask", blue_mask);
        cv::Mat contours_img;
        contours_img = img_in.clone();
        cv::drawContours(contours_img, contours, -1, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
        std::cout << "N. contours: " << contours.size() << std::endl;
#endif

        std::vector<cv::Point> approx_curve;
        std::vector<std::vector<cv::Point>> contours_approx;
        bool found = false;
        for (int i = 0; i < contours.size(); ++i) {
            // Approximate the i-th contours
            cv::approxPolyDP(contours[i],
                             approx_curve, 30, true);

            // Check the number of edge of the aproximate contour
            if (approx_curve.size() != 3) continue;

            // If we want to chech the area of the poliygon
            double area = cv::contourArea(approx_curve);


#ifdef FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined
            std::cout << (i + 1) << ") Contour size: " << contours[i].size() << std::endl;
            std::cout << (i + 1) << ") Aprox Contour size: " << approx_curve.size() << std::endl;
            std::cout << "Area: " << area << std::endl;
            contours_approx = {approx_curve};

            cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
#endif

            found = true;  // we found the blue triangle exit!
            break;
        }

        // If we found the robot triangle set robot position and create the triangle poligon
        if (found) {
            // emplace back every vertex on triangle (output of this function)
            for (const auto &pt: approx_curve) {
                triangle.emplace_back(pt.x / scale, pt.y / scale);
                // remember to use the scale to convert the position on the image
                // (pixels) to the position in the arena (meters)
            }

            // Find the position of the robot
            // NB: the position of the robot coincide with the baricenter of the triangle
            double cx = 0, cy = 0;

            // Compute the triangle baricenter
            for (auto vertex: triangle) {
                // NB: triangle point are expressed in meters
                cx += vertex.x;
                cy += vertex.y;
            }
            cx /= static_cast<double>(triangle.size());
            cy /= static_cast<double>(triangle.size());

            // Find the robot orientation (i.e the angle of height relative to the base with the x axis)
            double dst = 0;
            Point top_vertex; //
            for (auto &vertex: triangle) {
                const double dx = vertex.x - cx;
                const double dy = vertex.y - cy;
                const double curr_d = dx * dx + dy * dy;
                if (curr_d > dst) {
                    dst = curr_d;
                    top_vertex = vertex;
                }
            }

            // Store the position of the robot in the output
            x = cx;
            y = cy;

            // Compute the robot orientation
            const double dx = cx - top_vertex.x;
            const double dy = cy - top_vertex.y;
            theta = std::atan2(dy, dx);

#ifdef FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined
            // Draw over the imag ethe ba
            cv::Point cv_baricenter(x * scale, y * scale); // convert back m to px
            cv::Point cv_vertex(top_vertex.x * scale, top_vertex.y * scale); // convert back m to px
            cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0, 255, 0), 3);
            cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0, 255, 0), -1);
            std::cout << "(x, y, theta) = " << x << ", " << y << ", " << theta * 180 / M_PI << std::endl;
#endif
        }

#ifdef FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined
        cv::imshow("findRobot", contours_img);
        cv::waitKey(1);
#endif

        return true;
        //return found;
    }


    void dubinsArcToPoseVector(DubinsArc arc, std::vector<Pose>& vector) {
        /**
         * npts = 100;
  pts = zeros(npts+1, 2);
  for j = 0:npts
    s = arc.L/npts * j;
    [x, y] = circline(s, arc.x0, arc.y0, arc.th0, arc.k);
    pts(j+1, 1:2) = [x, y];
  end
         */
         /*

        DubinsArc temp;
        Pose pose;
        circleLine(0, arc.x0, arc.y0, arc.th0, arc.k, &temp);
        vector.push_back(pose);
        */

        const int numPoints = 100;
        for (int i = 0; i < numPoints; i++) {
            DubinsArc temp;
            Pose pose;
            double s = arc.L / numPoints * i;
            circleLine(s, arc.x0, arc.y0, arc.th0, arc.k, &temp);
            pose.x = temp.x0;
            pose.y = temp.y0;
            std::cout << temp.x0 << std::endl;
            vector.emplace_back(1, temp.xf, temp.yf, temp.thf, temp.k);
        }
    }


    std::vector<Pose> dubinsCurveToPoseVector(DubinsCurve curve) {
        std::vector<Pose> vector;
        dubinsArcToPoseVector(curve.a1, vector);
        dubinsArcToPoseVector(curve.a2, vector);
        dubinsArcToPoseVector(curve.a3, vector);
        return vector;
    }

    bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,
                  const std::vector<std::pair<int,Polygon>>& victim_list,
                  const Polygon& gate, const float x, const float y, const float theta,
                  Path& path,
                  const std::string& config_folder) {
        std::cout << "planPath called" << std::endl;

        RobotPosition start(0, 0, (-2.0 / 3.0 * M_PI));
        RobotPosition end(4, 0, (M_PI / 3.0));
        double kMax = 3;

        auto res = dubinsShortestPath(start, end, kMax);

        auto points = dubinsCurveToPoseVector(res);
        path.setPoints(points);

/*
        std::vector<Pose> points;
        for (int i = 0; i < 10; i++) {
            points.emplace_back(1, i, i, 1, 1);
        }
        path.setPoints(points);
*/
        return true;
    }
}

