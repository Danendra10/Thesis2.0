#include "main/main.hpp"

using namespace cv;
using namespace std;

const int goal_x = 400;
const int goal_y = 500;
const int FIELD_WIDTH = 1400;
const int FIELD_HEIGHT = 1000;
const double KAPPA = 0.01; // Attractive constant. You can adjust as needed.
const double CELL_SIZE = 10.0;

/**
 * @brief Draw the field
 */
float CalculateAttractivePotential(Point2f point, Point2f target)
{
}

int main()
{
    const int N = 1000;       // Number of points
    const double step = 0.05; // Step between points
    Mat points(N, 1, CV_32FC3);

    // Create a viz window
    viz::Viz3d window("Potential Sim");

    // Display the X, Y, and Z axes
    viz::WCoordinateSystem coords(100.0); // The parameter defines the length of the axes
    window.showWidget("Coordinate System", coords);

    DrawField(window);
    // DrawOuterLine(window);
    DrawBox(window, Point3d(robot_1_own.pose.x, robot_1_own.pose.y, 0), "Robot Own");
    DrawBox(window, Point3d(enemy_robot_1.pose.x, enemy_robot_1.pose.y, 0), "Robot Enemy", viz::Color::cyan());

    // Parameters for the potential field calculation
    Point2f goal(goal_x, goal_y);

    // Compute the potential field over the entire field
    Mat potentialField(FIELD_HEIGHT, FIELD_WIDTH, CV_32FC1);
    for (int y = 0; y < FIELD_HEIGHT; y++)
    {
        for (int x = 0; x < FIELD_WIDTH; x++)
        {
            float distance = cv::norm(cv::Point2f(x, y) - goal);
            potentialField.at<float>(y, x) = 0.5 * KAPPA * distance * distance;
        }
    }

    cv::normalize(potentialField, potentialField, 0, 1, cv::NORM_MINMAX);

    Mat grad_x, grad_y;
    Sobel(potentialField, grad_x, CV_32F, 1, 0, 3);
    Sobel(potentialField, grad_y, CV_32F, 0, 1, 3);

    window.spin();

    return 0;
}