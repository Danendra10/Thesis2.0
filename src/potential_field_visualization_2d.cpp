#include <opencv2/opencv.hpp>

const int GRID_SIZE = 1000;
const double k_attr = 100;
const double k_rep = 50000000.0;
const double d0 = 100.0;

cv::Point goal(800, 500);
const int DISPLAY_GRID_SIZE = 10;

double attractivePotential(const cv::Point &point)
{
    double d = cv::norm(point - goal);
    return 0.5 * k_attr * d * d;
}

double repulsivePotential(const cv::Point &point, const cv::Point &obstacle)
{
    double d = cv::norm(point - obstacle);
    if (d < d0)
    {
        return (0.5 * k_rep * (1.0 / d - 1.0 / d0) * (1.0 / d - 1.0 / d0) != INFINITY) ? 0.5 * k_rep * (1.0 / d - 1.0 / d0) * (1.0 / d - 1.0 / d0) : 0;
    }
    return 0;
}

int main()
{
    cv::Mat potentialField(GRID_SIZE, GRID_SIZE, CV_64F);
    cv::Point obstacle(500, 100);

    double maxU_rep = 0;

    // First loop: Compute the potentials and find max repulsive potential
    for (int x = 0; x < GRID_SIZE; x++)
    {
        for (int y = 0; y < GRID_SIZE; y++)
        {
            cv::Point point(x, y);
            double U_rep = repulsivePotential(point, obstacle);
            maxU_rep = std::max(maxU_rep, U_rep);
        }
    }

    // Second loop: Normalize and store the potentials in the potentialField
    for (int x = 0; x < GRID_SIZE; x++)
    {
        for (int y = 0; y < GRID_SIZE; y++)
        {
            cv::Point point(x, y);
            double U_rep = repulsivePotential(point, obstacle);
            U_rep = (maxU_rep != 0) ? U_rep / maxU_rep : 0; // Normalize
            U_rep *= 100;                                   // Then multiply by 100
            potentialField.at<double>(y, x) = U_rep;
        }
    }

    // Visualization
    cv::Mat displayImage;
    cv::normalize(potentialField, potentialField, 0, 1, cv::NORM_MINMAX);
    potentialField.convertTo(displayImage, CV_8U, 255);
    cv::applyColorMap(displayImage, displayImage, cv::COLORMAP_JET);
    // ... [rest of the visualization code remains unchanged]

    cv::imshow("Potential Field", displayImage);
    cv::waitKey(0);

    return 0;
}
