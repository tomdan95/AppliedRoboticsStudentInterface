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
    static bool autoFindArenaEdges = true; //change to false to use old manual arena edges selector.

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

    vector<cv::Point2f> findArenaEdge(const cv::Mat &img) {
        result.clear();
        cv::Mat hsv_img, black_mask, final_img;
        //change img from rgb to hsv
        cv::cvtColor(img, hsv_img, cv::COLOR_RGB2HSV);
        //clear all color but black
        cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 30), black_mask);
        
        //find arena contours
        vector<vector<cv::Point>> contours;
        cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        //aproximate arena contours
        int approxPolyEpsilon = 10; //aproximation value
        vector<vector<cv::Point>> approximatedContours;
        for (vector<cv::Point> contour : contours) {
            vector<cv::Point> approximatedContour;
            cv::approxPolyDP(contour, approximatedContour, 10, true);
            approximatedContours.push_back(approximatedContour);
        }

        //draw aproximated arena c  ontours on finalimg
        final_img = img.clone();
        cv::drawContours(final_img, approximatedContours, -1, cv::Scalar(255, 0, 0), 10);

        //recover and draw arena edges form arenacontours
        for (auto &contour : approximatedContours) {
            for (int i=0; i<contour.size(); i++) {
                int a=(i+2)%4;
                result.emplace_back(contour[a].x, contour[a].y);
                cv::circle(final_img, cv::Point(contour[a].x, contour[a].y), 10*(i+1), cv::Scalar(0, 0, 255), -1);
            }
        }

        //show immage user check and return found edges
        name = "Check if correct";
        //int i = sizeof(approximatedContours);
        //name = std::to_string(i);
        cv::imshow(name.c_str(), final_img);
        cv::namedWindow(name.c_str());
        cv::waitKey(0);
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
        if(autoFindArenaEdges){
            edges = findArenaEdge(img_in);
        }
        else{
            if (!alreadyDidExtrinsicCalibBefore(file_path)) {
                edges = askUserToClickArenaEdges(img_in);
                storeArenaEdgesToFile(config_folder, file_path, edges);
            } else {
                edges = loadArenaEdgesFromFile(file_path);
            }
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