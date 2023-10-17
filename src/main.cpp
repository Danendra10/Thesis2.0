#include "main/main.hpp"

using namespace cv;
using namespace std;

const int target_x = 400;
const int target_y = 500;

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
    const int FIELD_WIDTH = 1400;
    const int FIELD_HEIGHT = 1000;
    const double KAPPA = 0.01; // Attractive constant. You can adjust as needed.
    const double CELL_SIZE = 10.0;
    Point2f target(target_x, target_y);

    Logger(RED, "STARTING THE CALCULATION...");
    // Compute the potential field over the entire field
    Mat potentialField(FIELD_HEIGHT, FIELD_WIDTH, CV_32FC1);
    for (int y = 0; y < FIELD_HEIGHT; y++)
    {
        for (int x = 0; x < FIELD_WIDTH; x++)
        {
            float distance = cv::norm(cv::Point2f(x, y) - target);
            potentialField.at<float>(y, x) = 0.5 * KAPPA * distance * distance;
        }
    }

    cv::normalize(potentialField, potentialField, 0, 1, cv::NORM_MINMAX);

    vector<viz::WCube> cubes;

    for (int i = 0; i < potentialField.rows; i++)
    {
        for (int j = 0; j < potentialField.cols; j++)
        {
            try
            {
                double x = i * CELL_SIZE;
                double y = j * CELL_SIZE;
                double potential = potentialField.at<float>(i, j);
                cv::viz::WCube cube(cv::Point3d(x, y, 0), cv::Point3d(x + CELL_SIZE, y + CELL_SIZE, potential * 100), true, cv::viz::Color(potential * 255, (1 - potential) * 255, 0));
                cubes.push_back(cube);
            }
            catch (const std::exception &e)
            {
                Logger(RED, "Error on main: %s", e.what());
            }
        }
        cout << "Row: " << i << endl;
    }

    for (int i = 0; i < cubes.size(); i++)
    {
        window.showWidget("Cube" + to_string(i), cubes[i]);
    }

    window.spin();

    return 0;
}