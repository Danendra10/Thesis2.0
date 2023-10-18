#include "main/main.hpp"

using namespace cv;

const int goal_x = 400;           // Target of the robot x
const int goal_y = 500;           // Target of the robot y
const int obs_1_x = 1000;         // Target of the robot x
const int obs_1_y = 300;          // Target of the robot y
const int FIELD_WIDTH = 1400;     // Width of the field
const int FIELD_HEIGHT = 1000;    // Height of the field
const int visualization_step = 5; // Visualization step
const float Kattr = 0.1;          // Attractive constant. You can adjust as needed.
const float Krep = 1000;          // Repulsive constant. You can adjust as needed.
const float repulsive_radius = 300;

double CalculateAttractivePotential(float x, float y, float target_x, float target_y)
{
    return 0.5 * Kattr * cv::norm(cv::Point2f(x, y) - cv::Point2f(target_x, target_y));
}

double CalculateRepulsivePotential(float x, float y, float obstacle_x, float obstacle_y)
{
    float distance = cv::norm(cv::Point2f(x, y) - cv::Point2f(obstacle_x, obstacle_y));
    if (distance < repulsive_radius)
    {
        return 0.5 * Krep * (1.0 / distance - 1.0 / repulsive_radius) * (1.0 / distance - 1.0 / repulsive_radius);
        return 100;
    }
    return 0;
}

std::vector<cv::Vec3b> computeColors(const std::vector<cv::Point3f> &points)
{
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::min();
    for (const auto &point : points)
    {
        if (point.z < minZ)
            minZ = point.z;
        if (point.z > maxZ)
            maxZ = point.z;
    }

    std::vector<cv::Vec3b> colors;
    for (const auto &point : points)
    {
        double normalized = (point.z - minZ) / (maxZ - minZ);
        colors.push_back(cv::Vec3b(255 * (1 - normalized), 0, 255 * normalized));
    }
    return colors;
}
int main()
{
    InitLogger();
    // Create a viz window
    viz::Viz3d window("Potential Sim");

    // Display the X, Y, and Z axes
    viz::WCoordinateSystem coords(100.0); // The parameter defines the length of the axes
    window.showWidget("Coordinate System", coords);

    DrawField(window);

    std::vector<cv::Point3f> points_attr;
    std::vector<cv::Point3f> points_repl;
    std::vector<cv::Point3f> points_tot;

    for (int i = 0; i < FIELD_WIDTH; i += visualization_step)
    {
        for (int j = 0; j < FIELD_HEIGHT; j += visualization_step)
        {
            double attractive_potential = CalculateAttractivePotential(i, j, goal_x, goal_y) * 10;
            double repulsive_potential = CalculateRepulsivePotential(i, j, obs_1_x, obs_1_y) * 10;

            double total_potential = attractive_potential;

            if (repulsive_potential != 0 && repulsive_potential != INFINITY)
                total_potential += repulsive_potential;

            // ------------------ 3D Variables ------------------
            points_attr.push_back(cv::Point3f(i, j, attractive_potential));
            if (repulsive_potential != INFINITY)
            {
                points_repl.push_back(cv::Point3f(i, j, repulsive_potential));
            }

            points_tot.push_back(cv::Point3f(i, j, total_potential));
        }
    }

    std::vector<cv::Vec3b> colorsTot = computeColors(points_tot);

    cv::viz::WCloud cloudWidgetAttr(points_attr, cv::viz::Color::orange_red());
    viz::WCloud cloudWidgetRepl(points_repl, cv::viz::Color::blue());
    viz::WCloud cloudWidgetTot(points_tot, colorsTot);

    // window.showWidget("points attr", cloudWidgetAttr);
    // window.showWidget("points repl", cloudWidgetRepl);
    window.showWidget("points tot", cloudWidgetTot);

    // Spin and wait until the window is closed
    window.spin();

    return 0;
}
