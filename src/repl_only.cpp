#include <iostream>
#include <cmath>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
// Constants
const double kb = 1.0;  // For example, you can change this value
const double d0 = 1.0;  // For example
const double m = 1.0;   // Mass of the robot
const double dt = 0.01; // Time step
const double ka = 1.0;  // For example

// Image dimensions and scale factor for visualization
const int imgSize = 500;
const double scaleFactor = 50.0; // Adjust to fit the points within the image dimensions

// Function to compute distance between two Points
double Distance(Point2f a, Point2f b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Function to compute repulsion acceleration based on position q and closest obstacle qc
Point2f ReplAccel(Point2f q, Point2f qc)
{
    Point2f a_rep = {0, 0};
    double d = Distance(q, qc);
    double factor = -kb * (1.0 / d - 1.0 / d0) / (d * d) / m;
    double norm = Distance(q, qc);

    a_rep.x = factor * (q.x - qc.x) / norm;
    a_rep.y = factor * (q.y - qc.y) / norm;

    return a_rep;
}

Point2f AttrAccel(Point2f q, Point2f qd)
{
    Point2f a_att = {0, 0};
    a_att.x = -ka * (q.x - qd.x);
    a_att.y = -ka * (q.y - qd.y);

    return a_att;
}

// Main function
int main()
{
    // Initial conditions
    Point2f q = {1, 1};  // Initial position of the robot
    Point2f qc = {2, 2}; // Initial position of the closest obstacle
    Point2f v = {0, 0};  // Initial velocity
    Point2f qd = {4, 4}; // goal position

    cv::namedWindow("Robot Path", cv::WINDOW_AUTOSIZE);
    cv::Mat pathImg(imgSize, imgSize, CV_8UC3, cv::Scalar(255, 255, 255));

    circle(pathImg, Point(q.x * scaleFactor, q.y * scaleFactor), 2, Scalar(0, 0, 0), -1);
    circle(pathImg, Point(qc.x * scaleFactor, qc.y * scaleFactor), 2, Scalar(255, 0, 0), -1);

    // Compute acceleration and integrate to get velocity
    for (int step = 0; step < 100; ++step)
    { // For example, 100 steps
        Point2f repl_accel = ReplAccel(q, qc);
        // v.x += repl_accel.x * dt;
        // v.y += repl_accel.y * dt;

        Point2f attr_accel = AttrAccel(q, qd);

        v.x += attr_accel.x * dt;
        v.y += attr_accel.y * dt;

        q.x += v.x;
        q.y += v.y;

        // For demonstration purposes, you can print the velocity
        cout << "Velocity at step " << step << ": (" << v.x << ", " << v.y << ") "
             << "On position: (" << q.x << ", " << q.y << ")" << endl;

        // Draw the current position of the robot
        circle(pathImg, Point(q.x * scaleFactor, q.y * scaleFactor), 2, Scalar(0, 0, 0), -1);
        imshow("Robot Path", pathImg);

        // delay 500ms
        waitKey(500);
    }

    waitKey(0);

    return 0;
}
