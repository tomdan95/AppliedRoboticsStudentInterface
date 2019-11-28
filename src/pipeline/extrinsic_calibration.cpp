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

using namespace std;

namespace student {

    //-------------------------------------------------------------------------
    //          EXTRINSIC CALIB IMPLEMENTATION
    //-------------------------------------------------------------------------

    // Definition of the function pickNPoints and the callback mouseCallback.
    // The function pickNPoints is used to display a window with a background
    // image, and to prompt the user to select n points on this image.
    static cv::Mat bg_img;
    static vector<cv::Point2f> result;
    static string name;
    static atomic<bool> done;
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

    vector<cv::Point2f> askUserToClickArenaEdges(const cv::Mat &img) {
        result.clear();
        cv::Size small_size(img.cols / show_scale, img.rows / show_scale);
        cv::resize(img, bg_img, small_size);
        //bg_img = img.clone();
        name = "Pick 4 points";
        cv::imshow(name.c_str(), bg_img);
        cv::namedWindow(name.c_str());
        n = 4;

        done.store(false);

        cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
        while (!done.load()) {
            cv::waitKey(500);
        }

        cv::destroyWindow(name.c_str());
        return result;
    }

    bool alreadyDidExtrinsicCalibBefore(string fileName) {
        return experimental::filesystem::exists(fileName);
    }

    void storeArenaEdgesToFile (string configFolder, string fileName, vector<cv::Point2f> edges) {
        experimental::filesystem::create_directories(configFolder);
        // SAVE POINT TO FILE
        // cout << "IMAGE POINTS: " << endl;
        // for (const auto pt: image_points) {
        //   cout << pt << endl;
        // }
        ofstream output(fileName);
        if (!output.is_open()) {
            throw runtime_error("Cannot write file: " + fileName);
        }
        for (const auto pt: edges) {
            output << pt.x << " " << pt.y << endl;
        }
        output.close();
    }

    vector<cv::Point2f> loadArenaEdgesFromFile (string fileName) {
        vector<cv::Point2f> edges;
        ifstream input(fileName);
        if (!input.is_open()) {
            throw runtime_error("Cannot read file: " + fileName);
        }
        while (!input.eof()) {
            double x, y;
            if (!(input >> x >> y)) {
                if (input.eof()) break;
                else {
                    throw runtime_error("Malformed file: " + fileName);
                }
            }
            edges.emplace_back(x, y);
        }
        input.close();
        return edges;
    }

    bool extrinsicCalib(const cv::Mat &img_in, vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                        cv::Mat &rvec, cv::Mat &tvec, const string &config_folder) {
        string file_path = config_folder + "/extrinsicCalib.csv";
        vector<cv::Point2f> edges;
        if (!alreadyDidExtrinsicCalibBefore(file_path)) {
            edges = askUserToClickArenaEdges(img_in);
            storeArenaEdgesToFile(config_folder, file_path, edges);
        } else {
            edges = loadArenaEdgesFromFile(file_path);
        }

        cv::Mat dist_coeffs;
        dist_coeffs = (cv::Mat1d(1, 4) << 0, 0, 0, 0, 0);
        bool ok = cv::solvePnP(object_points, edges, camera_matrix, dist_coeffs, rvec, tvec);

        // cv::Mat Rt;
        // cv::Rodrigues(rvec_, Rt);
        // auto R = Rt.t();
        // auto pos = -R * tvec_;

        if (!ok) {
            cerr << "FAILED SOLVE_PNP" << endl;
        }

        return ok;
    }
}