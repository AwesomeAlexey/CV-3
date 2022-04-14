#include <opencv2/opencv.hpp>
#include <iostream>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <vector>
void plane_pics(void);
void thermal_camera(void);
void robots(void);
void qualify_wrenches(void);

using namespace std;
using namespace cv;

static inline long get_distance(const Point& a, const Point& b);


int main() {
    plane_pics();
    thermal_camera();
    robots();
    qualify_wrenches();
    return 0;
}



void plane_pics(void){

    for(const auto& i:{1, 2, 3}){

        string filename = string("../plane_pics/").append(to_string(i)).append(".jpg");

        cout << filename << endl;

        Mat img = imread(filename, IMREAD_GRAYSCALE);

        imshow(to_string(i), img);


        Mat res;
        threshold(img, res, 220, 300, THRESH_BINARY);
        erode(res, res, getStructuringElement(2, {3, 3}), {-1, 1}, 2);
        dilate(res, res, getStructuringElement(2, {6, 6}), {-1, 1}, 3);
        vector<vector<Point>> contours;
        findContours(res, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        vector <Point> centers;

        for(const auto& contour:contours){
            Moments m = moments(contour);
            Point c;
            c.x = int(m.m10/m.m00);
            c.y = int(m.m01/m.m00);
            centers.push_back(c);
            cout << c.x << " " << c.y << endl;
            circle(img, c, 12, {0}, -1);
            circle(img, c, 3, {255}, -1);
        }


        imshow(string("Res-").append(to_string(i)), img);
        imwrite(string("../plane_pics/Result-").append(to_string(i)).append(".jpg"), img);

        waitKey(0);

    }

}



void thermal_camera(void){

    for(const auto& i:{1, 2, 3, 4, 5}){

        string filename;
        if (i == 4)
            filename = string("../thermal_camera/").append(to_string(i)).append(".png");
        else
            filename = string("../thermal_camera/").append(to_string(i)).append(".jpg");
        Mat img = imread(filename);
        imshow(to_string(i), img);
        Mat hsv;
        cvtColor(img, hsv, COLOR_BGR2HSV);
        Scalar hsv_low = {0, 50, 150};
        Scalar hsv_high = {30, 255, 255};
        Mat res;
        inRange(hsv, hsv_low, hsv_high, res);

        erode(res, res, getStructuringElement(2, {3, 3}), {-1, 1}, 1);
        dilate(res, res, getStructuringElement(2, {6, 6}), {-1, 1}, 3);

        vector<vector<Point>> contours;
        findContours(res, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        vector <Point> centers;

        for(const auto& contour:contours){
            Moments m = moments(contour);
            Point c;
            c.x = int(m.m10/m.m00);
            c.y = int(m.m01/m.m00);
            centers.push_back(c);
            cout << c.x << " " << c.y << endl;
            circle(img, c, 12, {0, 0, 0}, -1);
            circle(img, c, 3, {255, 255, 255}, -1);
        }



        imshow(string("Res-").append(to_string(i)), img);
        imwrite(string("../thermal_camera/Result-").append(to_string(i)).append(".jpg"), img);


        waitKey(0);
        }
}



void robots(void){

    Mat img = imread("../robots/robots.jpg");
    imshow("Original", img);
    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);


    Scalar red_low = {0, 50, 150};
    Scalar red_high = {30, 255, 255};

    Scalar green_low = {50, 50, 150};
    Scalar green_high = {80, 255, 255};

    Scalar blue_low = {80, 50, 150};
    Scalar blue_high = {120, 255, 255};

    Scalar lamp_low = {0, 0, 150};
    Scalar lamp_high = {180, 5, 255};

    enum color{
        LAMP = 0, RED, GREEN, BLUE
    };

    vector<pair<pair<Scalar, Scalar>, Scalar>> colors = {
        {{lamp_low, lamp_high}, {255, 255, 255}},
        {{red_low, red_high}, {0, 0, 255}},
        {{green_low, green_high}, {0, 255, 0}},
        {{blue_low, blue_high}, {255, 0, 0}},
    };

    vector<vector<vector<Point>>> contours(4);
    vector<vector<Point>> centers(4);
    Point closest[3];

    for (const auto& i:{0, 1, 2, 3}){
        Mat res;
        inRange(hsv, colors[i].first.first, colors[i].first.second , res);
        erode(res, res, getStructuringElement(2, {2, 2}), {-1, 1}, 3);
        dilate(res, res, getStructuringElement(2, {12, 7}), {-1, 1}, 3);
//    erode(res, res, getStructuringElement(2, {6, 6}), {-1, 1}, 3);
        dilate(res, res, getStructuringElement(2, {5, 4}), {-1, 1}, 1);

        findContours(res, contours[i], RETR_EXTERNAL, CHAIN_APPROX_NONE);

        int index = 0;

        for (const auto& contour: contours[i]){
            Moments m = moments(contour);
            Point c;
            c.x = int(m.m10/m.m00);
            c.y = int(m.m01/m.m00);
            centers[i].push_back(c);
            if (i > 0) {
                if (centers[i].size() > 1) {
                    if (get_distance(closest[i - 1], centers[0][0]) > get_distance(c, centers[0][0])) {
                        closest[i - 1] = c;
                    }
                } else {
                    closest[i-1] = c;
                }
            }

//            cout << c.x << " " << c.y << endl;
            circle(img, c, 12, {0, 0, 0}, -1);
            circle(img, c, 3, colors[i].second, -1);
            drawContours(img, contours[i], index, {0, 0, 0}, 12);
            drawContours(img, contours[i], index, colors[i].second, 1);

            index++;
        }

        if (i) {
            cout << closest[i - 1] << endl;
            line(img, closest[i - 1], centers[0][0], colors[i].second, 3);
        }
        imshow("Original", img);
        imwrite(string("../robots/Result.jpg"), img);

        waitKey(0);

    }

    waitKey(0);

}

static inline long get_distance(const Point& a, const Point& b){

    return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);

}




void qualify_wrenches(void){
    auto wrenches = imread("../wrenches/wrenches.jpg");
    auto wrench = imread("../wrenches/wrench.jpg", IMREAD_GRAYSCALE);

    imshow("Wrenches", wrenches);

    Mat grayscale;
    cvtColor(wrenches, grayscale, COLOR_BGR2GRAY);

    Mat res;
    threshold(grayscale, res, 250, 255, THRESH_BINARY_INV);
    threshold(wrench, wrench, 250, 255, THRESH_BINARY);

    erode(res, res, getStructuringElement(2, {3, 3}), {-1, 1}, 3);
    dilate(res, res, getStructuringElement(2, {3, 3}), {-1, 1}, 3);

    vector<vector<Point>> contours;

    vector<vector<Point>> reference;

    findContours(res, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(wrench, reference, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    vector<Point> centers;
    int index = 0;

    for(const auto& contour: contours){
        auto match = matchShapes(reference[0], contour, ShapeMatchModes::CONTOURS_MATCH_I2, 0);
        if (match < 0.5)
            drawContours(wrenches, contours, index, {0, 255, 0}, 12);
        else
            drawContours(wrenches, contours, index, {0, 0, 255}, 12);

        index++;
        imshow("Wrenches", wrenches);
        waitKey(0);
    }

    imwrite("../wrenches/Result.jpg", wrenches);

}