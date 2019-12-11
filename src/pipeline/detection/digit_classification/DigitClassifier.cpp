#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "DigitClassifier.h"
#include "../../../opencv-utils.h"


vector<int> DigitClassifier::recognizeDigits(const cv::Mat &hsvImage, const cv::Mat &filteredImage, const vector<cv::Rect> &rects) const {
    cv::Mat preprocessedImage = preprocessImage(filteredImage);
    vector<int> digits;
    for (const cv::Rect &rect : rects) {
        int digit = recognizeDigit(hsvImage, preprocessedImage, rect);
        digits.push_back(digit);
    }
    return digits;
}


cv::Mat DigitClassifier::preprocessImage(const cv::Mat &imageWithDigits) {
    // TODO: Split in smooth and inverse image functions

    // Apply some filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1 * 2) + 1, (1 * 2) + 1));
    // Dilate using the generated kernel
    cv::dilate(imageWithDigits, imageWithDigits, kernel);
    // Erode using the generated kernel
    cv::erode(imageWithDigits, imageWithDigits, kernel);


    cv::Mat inversedImage;
    cv::bitwise_not(imageWithDigits, inversedImage);

    return inversedImage;
}


int DigitClassifier::recognizeDigit(const cv::Mat &hsvImage, const cv::Mat &preprocessedImage, const cv::Rect &rect) const {



    // Init a matrix specify its dimension (img.rows, img.cols), default color(255,255,255)
    // and elemet type (CV_8UC3).
    cv::Mat filtered(preprocessedImage.rows, preprocessedImage.cols, CV_8UC3, cv::Scalar(255, 255, 255));

    // Why can't we use directly preprocessedimage?
    hsvImage.copyTo(filtered, preprocessedImage);   // create copy of image without green shapes


    // create a 3x3 recttangular kernel for img filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2 * 2) + 1, (2 * 2) + 1));

    // Constructor of mat, we pass the original image and the coordinate to copy and we obtain
    // an image pointing to that subimage
    // TODO: This is done for every digit. Can we do it only once?
    cv::Mat processROI(filtered, rect); // extract the ROI containing the digit


    if (processROI.empty()) {
        cout << "processROI empty" << endl;
        return -1;
    }

    // The size of the number in the Template image should be similar to the dimension
    // of the number in the ROI
    cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
    cv::threshold(processROI, processROI, 100, 255,
                  0);   // threshold and binarize the image, to suppress some noise

    // Apply some additional smoothing and filtering
    cv::erode(processROI, processROI, kernel);
    cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
    cv::erode(processROI, processROI, kernel);

    // Find the template digit with the best matching
    double maxScore = 0;
    int digit = -1;
    for (const auto &digitTemplate: templates) {
        cv::Mat result;

        // Match the ROI with the templROIs j-th
        cv::matchTemplate(processROI, digitTemplate.first, result, cv::TM_CCOEFF);
        double score;
        cv::minMaxLoc(result, nullptr, &score);

        // Compare the score with the others, if it is higher save this as the best match!
        if (score > maxScore) {
            maxScore = score;
            digit = digitTemplate.second;
        }
    }

    return digit;
}


vector<pair<cv::Mat, int>> DigitClassifier::loadTemplates() {
    std::string template_folder = "/home/lar2019/robot/AppliedRoboticsStudentInterface/src/victim_templates/";
    vector<pair<cv::Mat, int>> templates;

    for (int i = 1; i <= 5; ++i) {
        cv::Mat templateImage = cv::imread(template_folder + std::to_string(i) + ".png");
        cv::flip(templateImage, templateImage, 1);
        for (int j = 0; j <= 8; j++) {
            cv::Mat image = templateImage.clone();
            cv::Mat rotatedImage = rotateImage(image, j * 45);
            templates.emplace_back(rotatedImage, i);
        }
    }

    return templates;
}

