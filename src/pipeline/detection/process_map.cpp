/**
 * To execute this function, set 'obstacle_detector' to false.
 */


#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "../../opencv-utils.h"
#include "find_victims.hpp"
#include "find_obstacles.hpp"
#include "find_gate.hpp"
#include "../DebugImage.h"
#include "../Config.h"


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
        GateDetector detector;
        vector<Polygon> gates = detector.findPolygons(hsv_img, scale);
        cout << "[PROCESS_MAP] Found " << gates.size() << " gates" << endl;
        if (gates.size()!=1) {
            return false;
        } else {
            gate = gates[0];
            return true;
        }
    }

    bool processVictims(const cv::Mat &rgbImage, const cv::Mat &hsvImage, const double scale, std::vector<std::pair<int, Polygon>> &victim_list, Config config) {

        VictimDetector detector(rgbImage, config);
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

    // TODO: Move up
    bool processMap(const cv::Mat &rgbImage, const double scale, vector<Polygon> &obstacleList,
                    vector<pair<int, Polygon>> &victimList, Polygon &gate, const string &configFolder) {
        Config config(configFolder);

        cv::Mat hsv_img = convertRGBToHSV(rgbImage);
        bool foundObstacles = processObstacles(hsv_img, scale, obstacleList);
        bool foundVictims = processVictims(rgbImage, hsv_img, scale, victimList, config);
        bool foundGate = processGate(hsv_img, scale, gate);
        return foundObstacles && foundVictims && foundGate;
    }

}