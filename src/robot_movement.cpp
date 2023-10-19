/**
 * TODO: DO TRANSLATE BASED ON THE POTENTIAL FIELD OUTPUT TO VELOCITY
 */
#include "main/main.hpp"

using namespace cv;
using namespace std;

// ------- Const Variables -------
const int goal_x = 400;           // Target of the robot x
const int goal_y = 500;           // Target of the robot y
const int FIELD_WIDTH = 1400;     // Width of the field
const int FIELD_HEIGHT = 1000;    // Height of the field
const int visualization_step = 5; // Visualization step
const float Kattr = 0.1;          // Attractive constant. You can adjust as needed.
const float Krep = 1000;          // Repulsive constant. You can adjust as needed.
const float repulsive_radius = 10;

// ------- Viz Variables -------
viz::Viz3d window("Robot Movement");

// ------- Utils Variables -------
uint8_t is_changing = 1;
uint8_t has_initialized = 0;
Affine3d robot_pose = Affine3d(Vec3d(robot_1_own.pose.x, robot_1_own.pose.y, 0));

void KeyboardCallback(const cv::viz::KeyboardEvent &event, void *viewer_void);
void KeyboardViz3d(const viz::KeyboardEvent &w, void *t);
void MoveRobot(RobotType &robot, float vel_x, float vel_y, float vel_theta);
int Initialize();
int Routine();

double CalculateAttractivePotential(float x, float y, float target_x, float target_y);
double CalculateRepulsivePotential(float x, float y, float obstacle_x, float obstacle_y);
std::vector<cv::Vec3b> computeColors(const std::vector<cv::Point3f> &points);

int main()
{
    if (Initialize() == -1)
    {
        printf("Initialize failed\n");
        return -1;
    }

    if (Routine() == -1)
    {
        Logger(RED, "Routine failed");
        return -1;
    }

    return 0;
}

//---------------------------------- Main ----------------------------------//
int Initialize()
{
    InitLogger();

    // Axis arrow for helper
    viz::WCoordinateSystem coords(100.0); // The parameter defines the length of the axes
    window.showWidget("Coordinate System", coords);

    window.registerKeyboardCallback(KeyboardCallback, (void *)&window);
}

int Routine()
{
    // Draw field
    DrawField(window);

    DrawBox(window, Point3d(robot_1_own.pose.x, robot_1_own.pose.y, 0), "Robot 1", viz::Color::blue());

    // Draw Enemy Robot
    DrawBox(window, Point3d(enemy_robot_1.pose.x, enemy_robot_1.pose.y, 0), "Enemy Robot 1", viz::Color::red());

    while (true)
    {
        if (is_changing)
        {
            printf("Robot 1: (%f, %f, %f)\n", robot_1_own.pose.x, robot_1_own.pose.y, robot_1_own.pose.th);
            std::vector<cv::Point3f> points_tot;

            for (int i = 0; i < FIELD_WIDTH; i += visualization_step)
            {
                for (int j = 0; j < FIELD_HEIGHT; j += visualization_step)
                {
                    double attractive_potential = CalculateAttractivePotential(i, j, goal_x, goal_y) * 10;
                    double repulsive_potential = CalculateRepulsivePotential(i, j, enemy_robot_1.pose.x, enemy_robot_1.pose.y) * 10;
                    // double repulsive_potential_2 = CalculateRepulsivePotential(i, j, obs_2_x, obs_2_y) * 10;

                    double total_potential = attractive_potential;

                    if (repulsive_potential != 0 && repulsive_potential != INFINITY)
                        total_potential += repulsive_potential;

                    // ------------------ 3D Variables ------------------
                    points_tot.push_back(cv::Point3f(i, j, total_potential));
                }
            }

            std::vector<cv::Vec3b> colorsTot = computeColors(points_tot);

            viz::WCloud cloudWidgetTot(points_tot, colorsTot);

            if (has_initialized)
                window.removeWidget("points tot");
            window.showWidget("points tot", cloudWidgetTot);

            is_changing = 0;
        }

        has_initialized = 1;

        window.spinOnce(1, true);
    }
}

//---------------------------------- Utils ----------------------------------//

void KeyboardCallback(const cv::viz::KeyboardEvent &event, void *viewer_void)
{
    cv::viz::Viz3d *viewer = static_cast<cv::viz::Viz3d *>(viewer_void);

    cv::Vec3d current_translation = robot_pose.translation();

    // cout << "Current translation: " << current_translation << endl;

    if (event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN)
    {
        if (event.symbol == "a")
        {
            current_translation[0] = -100;
            current_translation[1] = 0;
        }
        else if (event.symbol == "d")
        {
            current_translation[0] = 100;
            current_translation[1] = 0;
        }
        else if (event.symbol == "w")
        {
            current_translation[0] = 0;
            current_translation[1] = 100;
        }
        else if (event.symbol == "s")
        {
            current_translation[0] = 0;
            current_translation[1] = -100;
        }
        is_changing = 1;

        viewer->updateWidgetPose("Robot 1", robot_pose);
        robot_pose.translation(current_translation);
    }
}

void MoveRobot(RobotType &robot, float vel_x = 0, float vel_y = 0, float vel_theta = 0)
{
    robot.pose.x += vel_x;
    robot.pose.y += vel_y;
    robot.pose.th += vel_theta;
    is_changing = 1;
}

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
        // return 100;
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