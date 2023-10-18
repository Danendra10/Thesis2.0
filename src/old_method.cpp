#include "potential_field_logic/vec_attr.hpp"
#include "potential_field_logic/vec_repl.hpp"
#include "entities/robot.hpp"
#include "logger/logger.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

const uint8_t repulsive_radius = 100;
const uint8_t attractive_radius = 100;
const uint8_t total_x_point_to_check = 31;
const uint8_t total_y_point_to_check = 31;
const uint16_t field_x_cfg = 1200;
const uint16_t field_y_cfg = 800;
const uint16_t goal_x = 700;
const uint16_t goal_y = 500;

RobotType robot_1(0, 0, 90, 0, 0, 0, OWN_ROBOT);
RobotType enemy_1(100, 100, 90, 0, 0, 0, ENEMY_ROBOT);

Mat frame(field_x_cfg, field_y_cfg, CV_8UC3, Scalar(0, 0, 0));

float RobotVelocity(float x, float y)
{
    return sqrt(x * x + y * y);
}

int main(int agrc, char **argv)
{
    InitLogger();
    VectorAttractive vec_attr;
    VectorRepulsive vec_repl;

    vec_attr.init(repulsive_radius, RobotVelocity(robot_1.vel.x, robot_1.vel.y));
    vec_repl.init(attractive_radius, RobotVelocity(robot_1.vel.x, robot_1.vel.y));

    for (uint8_t x = 0; x < total_x_point_to_check; x++)
    {
        for (uint8_t y = 0; y < total_y_point_to_check; y++)
        {
            uint16_t x_thresh_range = field_x_cfg / total_x_point_to_check;
            uint16_t y_thresh_range = field_y_cfg / total_y_point_to_check;

            float pos_x = x * x_thresh_range;
            float pos_y = y * y_thresh_range;

            float v_x = pos_x;
            float v_y = pos_y;
            float r, theta;

            vec_attr.update(pos_x, pos_y, goal_x, goal_y, r, theta);
            v_x += r * cos(theta);
            v_y += r * sin(theta);

            Logger(RED, "x: %f, y: %f, r: %f, theta: %f", pos_x, pos_y, r, theta);
            Logger(RED, "v_x: %f, v_y: %f", v_x, v_y);
        }
    }

    while (true)
    {
        imshow("Field", frame);
        waitKey(1);
    }
}