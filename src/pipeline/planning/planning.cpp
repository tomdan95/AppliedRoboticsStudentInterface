/**
 * To execute this function, set 'planning' to false.
 */
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "dubins/temp.h"
#include "dubins/dubins.h"
#include "dubins/curve.h"


namespace student {

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
        RobotPosition end(1, 1, (M_PI / 3.0));
        double kMax = 5;

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