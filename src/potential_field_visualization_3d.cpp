#include <opencv2/viz/vizcore.hpp>
#include <cmath>

const int GRID_SIZE = 100;
const double k_attr = 0.5;
const double k_rep = 0.5;
const double d0 = 10.0;

cv::Point3d goal(50, 50, 0); // set some goal in the middle

double attractivePotential(const cv::Point3d &point)
{
    double d = cv::norm(point - goal);
    return 0.5 * k_attr * d * d;
}

double repulsivePotential(const cv::Point3d &point, const cv::Point3d &obstacle)
{
    double d = cv::norm(point - obstacle);
    if (d < d0)
    {
        return 0.5 * k_rep * (1.0 / d - 1.0 / d0) * (1.0 / d - 1.0 / d0);
    }
    return 0;
}

int main()
{
    cv::viz::Viz3d myWindow("Potential Fields Visualization");

    // Create a dummy obstacle
    cv::Point3d obstacle(30, 30, 0);

    for (int x = 0; x < GRID_SIZE; x++)
    {
        for (int y = 0; y < GRID_SIZE; y++)
        {
            cv::Point3d point(x, y, 0);
            double U_attr = attractivePotential(point);
            double U_rep = repulsivePotential(point, obstacle);

            cv::Point3d force_direction = (goal - point) * U_attr + (point - obstacle) * U_rep;
            force_direction *= 0.01; // Scale for visualization purposes

            // Visualize the force as an arrow
            cv::viz::WArrow arrow(point, point + force_direction, 0.5, cv::viz::Color::blue());
            myWindow.showWidget("Arrow" + std::to_string(x) + "_" + std::to_string(y), arrow);
        }
    }

    while (!myWindow.wasStopped())
    {
        myWindow.spinOnce(1, true);
    }

    return 0;
}
