/**
 * This file executes the extrinsic calib, given the coordinates of the 4 corners of the arena.
 * To execute this function, set 'extrinsic_calib' to false.
 */
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <atomic>
#include <zconf.h>
#include <experimental/filesystem>

namespace student {

    //-------------------------------------------------------------------------
    //          EXTRINSIC CALIB IMPLEMENTATION
    //-------------------------------------------------------------------------

    // Definition of the function pickNPoints and the callback mouseCallback.
    // The function pickNPoints is used to display a window with a background
    // image, and to prompt the user to select n points on this image.
    static cv::Mat bg_img;
    static std::vector<cv::Point2f> result;
    static std::string name;
    static std::atomic<bool> done;
    static int n;
    static double show_scale = 1.0;

    void mouseCallback(int event, int x, int y, int, void *p) {
        if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;

        result.emplace_back(x * show_scale, y * show_scale);
        cv::circle(bg_img, cv::Point(x, y), 20 / show_scale, cv::Scalar(0, 0, 255), -1);
        cv::imshow(name.c_str(), bg_img);

        if (result.size() >= n) {
            usleep(500 * 1000);
            done.store(true);
        }
    }

    std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat &img) {
        result.clear();
        cv::Size small_size(img.cols / show_scale, img.rows / show_scale);
        cv::resize(img, bg_img, small_size);
        //bg_img = img.clone();
        name = "Pick " + std::to_string(n0) + " points";
        cv::imshow(name.c_str(), bg_img);
        cv::namedWindow(name.c_str());
        n = n0;

        done.store(false);

        cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
        while (!done.load()) {
            cv::waitKey(500);
        }

        cv::destroyWindow(name.c_str());
        return result;
    }

    bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                        cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder) {

        std::string file_path = config_folder + "/extrinsicCalib.csv";

        std::vector<cv::Point2f> image_points;

        if (!std::experimental::filesystem::exists(file_path)) {

            std::experimental::filesystem::create_directories(config_folder);

            image_points = pickNPoints(4, img_in);
            // SAVE POINT TO FILE
            // std::cout << "IMAGE POINTS: " << std::endl;
            // for (const auto pt: image_points) {
            //   std::cout << pt << std::endl;
            // }
            std::ofstream output(file_path);
            if (!output.is_open()) {
                throw std::runtime_error("Cannot write file: " + file_path);
            }
            for (const auto pt: image_points) {
                output << pt.x << " " << pt.y << std::endl;
            }
            output.close();
        } else {
            // LOAD POINT FROM FILE
            std::ifstream input(file_path);
            if (!input.is_open()) {
                throw std::runtime_error("Cannot read file: " + file_path);
            }
            while (!input.eof()) {
                double x, y;
                if (!(input >> x >> y)) {
                    if (input.eof()) break;
                    else {
                        throw std::runtime_error("Malformed file: " + file_path);
                    }
                }
                image_points.emplace_back(x, y);
            }
            input.close();
        }

        cv::Mat dist_coeffs;
        dist_coeffs = (cv::Mat1d(1, 4) << 0, 0, 0, 0, 0);
        bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

        // cv::Mat Rt;
        // cv::Rodrigues(rvec_, Rt);
        // auto R = Rt.t();
        // auto pos = -R * tvec_;

        if (!ok) {
            std::cerr << "FAILED SOLVE_PNP" << std::endl;
        }

        return ok;
    }
}