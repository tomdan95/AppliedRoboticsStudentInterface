#ifndef STUDENT_PROECT_DIGITCLASSIFIER_H
#define STUDENT_PROECT_DIGITCLASSIFIER_H


#include <opencv2/core/mat.hpp>
#include <utils.hpp>
#include "../../Config.h"

using namespace std;
using namespace student;

class DigitClassifier {

private:
    const vector<pair<cv::Mat, int>> templates;

    static vector<pair<cv::Mat, int>> loadTemplates(Config config);
    static cv::Mat preprocessImage(const cv::Mat &imageWithDigits);

    int recognizeDigit(const cv::Mat &hsvImage, const cv::Mat &preprocessedImage, const cv::Rect &rect) const;

public:

    DigitClassifier(Config config) : templates(loadTemplates(config)) {}

    /**
     *
     * @param filteredImage color filtered image
     * @param polygons
     * @return
     */
    vector<int> recognizeDigits(const cv::Mat &hsvImage, const cv::Mat &filteredImage, const vector<cv::Rect> &rects) const;
};


#endif //STUDENT_PROECT_DIGITCLASSIFIER_H
