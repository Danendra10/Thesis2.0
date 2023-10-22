/**
 * TODO: DO TRANSLATE BASED ON THE POTENTIAL FIELD OUTPUT TO VELOCITY, DO THE ATTRACTIVE FIRST
 *
 */
#include "main/main.hpp"

// #define VISUALIZE
#define AUTO_MOVE

using namespace cv;
using namespace std;

// ------- Const Variables -------
const int goal_x = 1400;          // Target of the robot x
const int goal_y = 1000;          // Target of the robot y
const int FIELD_WIDTH = 1400;     // Width of the field
const int FIELD_HEIGHT = 1000;    // Height of the field
const int visualization_step = 5; // Visualization step
const float Kattr = 0.1;          // Attractive constant. You can adjust as needed.
const float Krep = 1000;          // Repulsive constant. You can adjust as needed.
const float repulsive_radius = 100;

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
void MoveRobotUsingPotentialField(Affine3d &robot);
void UpdateCameraToFollowRobot(viz::Viz3d &window, const Affine3d &robot_pose);

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
#ifdef VISUALIZE
        if (is_changing)
        {

            std::vector<cv::Point3f> points_tot;

            for (int i = -200; i < FIELD_WIDTH + 200; i += visualization_step)
            {
                for (int j = -200; j < FIELD_HEIGHT + 200; j += visualization_step)
                {
                    double attractive_potential = CalculateAttractivePotential(i, j, goal_x, goal_y) * 10;
                    double repulsive_potential = CalculateRepulsivePotential(i, j, enemy_robot_1.pose.x, enemy_robot_1.pose.y) * 10;
                    // double repulsive_potential_2 = CalculateRepulsivePotential(i, j, obs_2_x, obs_2_y) * 10;

                    double total_potential = attractive_potential;

                    if (repulsive_potential != 0 && repulsive_potential != INFINITY)
                        total_potential += repulsive_potential;

                    // when i and j is outside the field, the potential is 1000
                    if (i < 0 || i > FIELD_WIDTH || j < 0 || j > FIELD_HEIGHT)
                        total_potential = 1000;
                    // ------------------ 3D Variables ------------------
                    points_tot.push_back(cv::Point3f(i, j, total_potential));
                }
            }

#endif
#ifdef AUTO_MOVE
            // Calculate the gradient of the attractive field
            // Point2f gradient(Kattr * (robot_1_own.pose.x - goal_x), Kattr * (robot_1_own.pose.y - goal_y));

            // double length = norm(gradient);
            // if (length > 0.1)
            // {
            //     gradient *= -0.1 / length;

            //     float theta = atan2(gradient.y, gradient.x);

            //     Logger(YELLOW, "Theta: %f, Gradient: (%f, %f), Current robot pose: (%f, %f)",
            //            theta, gradient.x, gradient.y,
            //            robot_1_own.pose.x, robot_1_own.pose.y);
            //     // robot_pose.linear(cv::Matx33d(cos(theta), -sin(theta), 0,
            //     //                               sin(theta), cos(theta), 0,
            //     //                               0, 0, 1));

            //     robot_pose.translation(cv::Vec3d(robot_pose.translation()[0] + gradient.x, robot_pose.translation()[1] + gradient.y, robot_pose.translation()[2]));

            //     robot_1_own.pose.th = theta;
            //     robot_1_own.pose.x = robot_pose.translation()[0];
            //     robot_1_own.pose.y = robot_pose.translation()[1];
            MoveRobotUsingPotentialField(robot_pose);
            UpdateCameraToFollowRobot(window, robot_pose);

            window.updateWidgetPose("Robot 1", robot_pose);
            // }
#endif
#ifdef VISUALIZE
            std::vector<cv::Vec3b> colorsTot = computeColors(points_tot);

            viz::WCloud cloudWidgetTot(points_tot, colorsTot);

            if (has_initialized)
                window.removeWidget("points tot");
            window.showWidget("points tot", cloudWidgetTot);

            is_changing = 0;
        }
#endif
        has_initialized = 1;

        window.spinOnce(1, true);
    }
}

//---------------------------------- Utils ----------------------------------//

void KeyboardCallback(const cv::viz::KeyboardEvent &event, void *viewer_void)
{
    cv::viz::Viz3d *viewer = static_cast<cv::viz::Viz3d *>(viewer_void);

    cv::Vec3d current_translation = robot_pose.translation();

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

void MoveRobot(Affine3d &robot, float vel_x = 0, float vel_y = 0, float vel_theta = 0)
{
    robot.translation(cv::Vec3d(vel_x, vel_y, robot.translation()[2]));
    is_changing = 1;
}

double CalculateDistanceBetweenTwoPoints(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double CalculateAttractivePotential(float x, float y, float target_x, float target_y)
{
    return 0.5 * Kattr * cv::norm(cv::Point2f(x, y) - cv::Point2f(target_x, target_y));
}

double CalculateRepulsivePotential(float x, float y, float obstacle_x, float obstacle_y)
{
    double distance = CalculateDistanceBetweenTwoPoints(x, y, obstacle_x, obstacle_y);
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

cv::Point2f ComputeGradient(float x, float y)
{
    float epsilon = 1.0; // Small value for numerical differentiation

    // Calculate potential for current position
    double U = CalculateAttractivePotential(x, y, goal_x, goal_y) + CalculateRepulsivePotential(x, y, enemy_robot_1.pose.x, enemy_robot_1.pose.y);

    // Calculate potential slightly shifted in x and y direction
    double U_x = CalculateAttractivePotential(x + epsilon, y, goal_x, goal_y) + CalculateRepulsivePotential(x + epsilon, y, enemy_robot_1.pose.x, enemy_robot_1.pose.y);
    double U_y = CalculateAttractivePotential(x, y + epsilon, goal_x, goal_y) + CalculateRepulsivePotential(x, y + epsilon, enemy_robot_1.pose.x, enemy_robot_1.pose.y);

    // Compute partial derivatives
    float dU_dx = (U_x - U) / epsilon;
    float dU_dy = (U_y - U) / epsilon;

    // cout << "dU_dx: " << dU_dx << " dU_dy: " << dU_dy << endl;

    return cv::Point2f(-dU_dx, -dU_dy); // Negative because we want to move against the gradient
}

void MoveRobotUsingPotentialField(Affine3d &robot_pose)
{
    static float pos_x = robot_pose.translation()[0];
    static float pos_y = robot_pose.translation()[1];

    cout << "POSITION: " << robot_pose.translation() << endl;

    cout << "ROBOT OWN: " << robot_1_own.pose.x << " " << robot_1_own.pose.y << endl;

    // Compute the gradient at the robot's current position
    cv::Point2f gradient = ComputeGradient(robot_pose.translation()[0], robot_pose.translation()[1]);
    float gradient_magnitude = cv::norm(gradient);
    cout << "Gradient: " << gradient << gradient_magnitude << endl;

    if (gradient_magnitude > 0.01)
    {
        // Normalize the gradient to get the direction
        cv::Point2f direction = gradient / gradient_magnitude;

        // Define the robot's speed
        float speed = 100;

        // Compute the velocity in the gradient direction
        cv::Point2f velocity = direction * speed;

        // Update the robot's position based on the velocity
        robot_pose.translation(cv::Vec3d(robot_pose.translation()[0] + velocity.x / 100,
                                         robot_pose.translation()[1] + velocity.y / 100,
                                         robot_pose.translation()[2]));

        robot_1_own.pose.x = robot_pose.translation()[0];
        robot_1_own.pose.y = robot_pose.translation()[1];

        pos_x += velocity.x / 100;
        pos_y += velocity.y / 100;

        cout << "posx: " << pos_x << " posy: " << pos_y << endl;
    }
}

void UpdateCameraToFollowRobot(viz::Viz3d &window, const Affine3d &robot_pose)
{
    // Define height from which camera looks down at the robot
    const double height = 1.0; // 1.0 meters above the robot

    // Calculate the camera position (directly above the robot)
    cv::Vec3d camPosition = robot_pose.translation() + cv::Vec3d(0, 0, height);

    // Set the camera's focal point to the robot's position
    cv::Vec3d focalPoint = robot_pose.translation();

    // Define camera's up direction to look downward (assuming y-axis points down)
    cv::Vec3d upDirection(0, -1, 0);

    // Update the window's camera view
    window.setViewerPose(viz::makeCameraPose(camPosition, focalPoint, upDirection));
}
