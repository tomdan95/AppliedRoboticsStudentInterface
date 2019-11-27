/**
 * To execute this function, set 'unwrap' to false.
 */
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

namespace student {
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
}