#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "DigitClassifier.h"
#include "../../../opencv-utils.h"


vector<int> DigitClassifier::recognizeDigits(const cv::Mat &image, const cv::Mat &hsvImage, const cv::Mat &filteredImage,
                                             const vector<cv::Rect> &rects) const {
    vector<int> digits;
    for (const cv::Rect &rect : rects) {
        int digit = recognizeDigit(image, hsvImage, filteredImage, rect);
        digits.push_back(digit);
    }
    return digits;
}

int DigitClassifier::recognizeDigit(const cv::Mat &image, const cv::Mat &hsvImage, const cv::Mat &greenMask,
                                    const cv::Rect &rect) const {


    // Apply some filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1 * 2) + 1, (1 * 2) + 1));
    cv::dilate(greenMask, greenMask, kernel);
    cv::erode(greenMask, greenMask, kernel);

    // Find contours
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    contours_img = image.clone();
    cv::findContours(greenMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    cv::Mat greenMaskInv;
    cv::Mat filtered(image.rows, image.cols, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::bitwise_not(greenMask,
                    greenMaskInv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask


    image.copyTo(filtered, greenMaskInv);   // create copy of image without green shapes

    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2 * 2) + 1, (2 * 2) + 1));


    cv::Mat processROI(filtered, rect); // extract the ROI containing the digit

    if (processROI.empty()) {
        return -1;
    }

    cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
    cv::threshold(processROI, processROI, 100, 255, 0); // threshold and binarize the image, to suppress some noise

    // Apply some additional smoothing and filtering
    cv::erode(processROI, processROI, kernel);
    cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
    cv::erode(processROI, processROI, kernel);

    // Show the actual image used for the template matching
    //showImageAndWaitKeyPress(processROI);


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


#define TEMPLATES_ROTATION_DEGREES  5
#define TEMPLATES_ROTATIONS         (360 / TEMPLATES_ROTATION_DEGREES)

vector<pair<cv::Mat, int>> DigitClassifier::loadTemplates(Config config) {
    vector<pair<cv::Mat, int>> templates;

    for (int i = 1; i <= 5; ++i) {
        string imageFileName = config.getNumberTemplatesFolder() + "/" + std::to_string(i) + ".png";
        cout << "loading template image " << imageFileName << endl;
        cv::Mat templateImage = cv::imread(imageFileName);
        if (templateImage.empty()) {
            throw runtime_error("can't load number templates to classify the number of the victims.");
        }
        cv::flip(templateImage, templateImage, 1);
        for (int j = 0; j < TEMPLATES_ROTATIONS; j++) {
            cv::Mat image = templateImage.clone();
            cv::Mat rotatedImage = rotateImage(image, j * TEMPLATES_ROTATION_DEGREES);
            templates.emplace_back(rotatedImage, i);
        }
    }

    return templates;
}

