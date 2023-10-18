#include <iostream>
#include <cmath>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "logger/logger.hpp"

using namespace cv;
using namespace std;

// Constants
const double ka = 2;    // For example
const double kb = 21.0; // For example, you can change this value
const double d0 = 1.0;  // For example
const double m = 1.0;   // Mass of the robot
const double dt = 0.01; // Time step

// Image dimensions and scale factor for visualization
const int imgSize = 500;
const double scaleFactor = 50.0; // Adjust to fit the points within the image dimensions

const double MAX_VELOCITY = 0.1; // Adjust this to set the maximum allowed speed

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
    double factor = kb * (1.0 / d - 1.0 / d0) / (d * d) / m;
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

void limitVelocity(Point2f &v)
{
    double magnitude = Distance(v, {0, 0});
    Logger(RED, "Magnitude: %f", magnitude);
    if (magnitude > MAX_VELOCITY)
    {
        v.x = (v.x / magnitude) * MAX_VELOCITY;
        v.y = (v.y / magnitude) * MAX_VELOCITY;
    }
}

// Main function
int main()
{
    // Initial conditions
    Point2f q = {1, 1};  // Initial position of the robot
    Point2f qc = {2, 1}; // Initial position of the closest obstacle
    Point2f v = {0, 0};  // Initial velocity
    Point2f qd = {4, 4}; // goal position

    cv::namedWindow("Robot Path", cv::WINDOW_AUTOSIZE);
    cv::Mat pathImg(imgSize, imgSize, CV_8UC3, cv::Scalar(255, 255, 255));

    circle(pathImg, Point(q.x * scaleFactor, q.y * scaleFactor), 2, Scalar(0, 0, 0), -1);
    circle(pathImg, Point(qc.x * scaleFactor, qc.y * scaleFactor), 2, Scalar(255, 0, 0), -1);
    circle(pathImg, Point(qd.x * scaleFactor, qd.y * scaleFactor), 2, Scalar(0, 0, 255), -1);

    // Compute acceleration and integrate to get velocity
    while (true)
    { // For example, 100 steps
        Point2f repl_accel = ReplAccel(q, qc);
        // Point2f repl_accel = {0, 0};
        Point2f attr_accel = AttrAccel(q, qd);
        // Point2f attr_accel = {0, 0};

        Point a_total;
        a_total.x = repl_accel.x + attr_accel.x;
        a_total.y = repl_accel.y + attr_accel.y;

        v.x += a_total.x * dt;
        v.y += a_total.y * dt;

        // limitVelocity(v);

        Logger(RED, "Velocity: %f %f", v.x, v.y);

        double speed = sqrt(v.x * v.x + v.y * v.y);

        if (speed > MAX_VELOCITY)
        {
            v.x = (v.x / speed) * MAX_VELOCITY;
            v.y = (v.y / speed) * MAX_VELOCITY;
        }

        q.x += v.x;
        q.y += v.y;

        // For demonstration purposes, you can print the velocity
        // cout << "Velocity at step "
        //      << ": (" << v.x << ", " << v.y << ") "
        //      << "On position: (" << q.x << ", " << q.y << ")" << endl;

        // Draw the current position of the robot
        circle(pathImg, Point(q.x * scaleFactor, q.y * scaleFactor), 2, Scalar(0, 0, 0), -1);
        imshow("Robot Path", pathImg);

        // delay 500ms
        waitKey(100);
    }

    waitKey(0);

    return 0;
}
