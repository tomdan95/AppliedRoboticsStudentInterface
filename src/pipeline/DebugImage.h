
#ifndef STUDENT_PROJECT_DEBUGIMAGE_H
#define STUDENT_PROJECT_DEBUGIMAGE_H


#include <opencv2/core/mat.hpp>
#include <utils.hpp>
#include "planning/Graph.h"

namespace student {

    class DebugImage {
    private:
        static cv::Mat image;

    public:
        static void showAndWait(int wait = 0);

        static void drawSegment(const Point &a, const Point &b, int multiply = 1);

        static void clear();

        static void drawGraph(student::Graph graph);

        static void drawImage(const cv::Mat &mat);
    };


}



#endif
