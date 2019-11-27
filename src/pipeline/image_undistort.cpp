/**
 * This file contains the function that undistort the image.
 * See slides relative to camera calibration.
 * To execute this function, set 'rectify' to false.
 */
#include "student_image_elab_interface.hpp"

using namespace std;

namespace student {

    // TODO: understand Shape_Detection_v2 slides
    void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                        const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const string &config_folder) {

        static bool mapInitialized = false;
        static cv::Mat full_map1, full_map2;

        if (!mapInitialized) {
            // Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
            cv::Mat R;
            cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix,
                                        img_in.size(), CV_16SC2, full_map1, full_map2);
            mapInitialized = true;
        }

        // Initialize output image
        cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);
    }
}