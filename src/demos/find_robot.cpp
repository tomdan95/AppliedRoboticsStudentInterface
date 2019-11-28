#define FIND_ROBOT_DEBUG_PLOT
#include "../pipeline/detection/find_robot.hpp"



cv::Mat loadImage(string fileName);
void showImageAndWaitKeyPress(cv::Mat &image);

int main() {
    cv::Mat img = loadImage("/home/lar2019/robot/AppliedRoboticsStudentInterface/src/areana_samples/000.jpg");
    showImageAndWaitKeyPress(img);
    Polygon triangle;
    double baricenterX, baricenterY;
    double theta;
    bool found = student::findRobot(img, 2, triangle, baricenterX, baricenterY, theta, "/tmp");

    if (found) {
        cout << "Found robot!" << endl;
        cout << "x: " << baricenterX << " y: " << baricenterY << " theta: " << theta << endl;
    } else {
        cout << "Robot not found!" << endl;
        return -1;
    }


    return 0;
}

cv::Mat loadImage(string fileName) {
    cv::Mat image = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);
    if (!image.data) {
        cout << "Could not open or find the image: " << fileName << endl;
        cout << "(If you're using a relative file path, check from which folder you're running this executable)" << endl;
        exit(-1);
    }
    return image;
}


void showImageAndWaitKeyPress(cv::Mat &image) {
    namedWindow("Loaded image", cv::WINDOW_AUTOSIZE);// Create a window for display.
    imshow("Display window", image);
    cv::waitKey(0);
}
