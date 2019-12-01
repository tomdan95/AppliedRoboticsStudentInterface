#ifndef STUDENT_PROECT_DIGITCLASSIFIER_H
#define STUDENT_PROECT_DIGITCLASSIFIER_H


#include <opencv2/core/mat.hpp>
#include <utils.hpp>

using namespace std;

class DigitClassifier {

private:
    const vector<pair<cv::Mat, int>> templates;

    static vector<pair<cv::Mat, int>> loadTemplates();
    static cv::Mat preprocessImage(const cv::Mat &imageWithDigits);

    int recognizeDigit(const cv::Mat &hsvImage, const cv::Mat &preprocessedImage, const cv::Rect &rect) const;

public:

    DigitClassifier() : templates(loadTemplates()) {}

    /**
     *
     * @param filteredImage color filtered image
     * @param polygons
     * @return
     */
    vector<int> recognizeDigits(const cv::Mat &hsvImage, const cv::Mat &filteredImage, const vector<cv::Rect> &rects) const;
};


#endif //STUDENT_PROECT_DIGITCLASSIFIER_H
