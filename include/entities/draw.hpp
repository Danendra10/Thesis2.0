#ifndef DRAW_HPP_
#define DRAW_HPP_

#include <opencv2/opencv.hpp>

#include "entities/field.hpp"
#include "entities/line.hpp"
#include "entities/goal_post.hpp"
#include "entities/robot.hpp"
#include "logger/logger.hpp"

#define CVWHITE Scalar(255, 255, 255)
#define CVRED Scalar(0, 0, 255)
#define CVROBOT Scalar(255, 0, 255)

#define Cm2PxX(x_) (50 + (x_))
#define Cm2PxY(y_) (50 + (y_))
#define Px2CmX(x_) ((x_)-50)
#define Px2CmY(y_) (50 - (y_))
#define DEG2RAD 0.017452925
#define RAD2DEG 57.295780

using namespace cv;

void DrawField(const Field &field, Mat &img)
{
    Logger(YELLOW, "Drawing field.. %d %d", field.length_y, field.length_y);
    rectangle(img, Point(0, 0), Point(field.length_y, field.length_y), field.base_color, -1);
}

void AddLine(const Line &line, Mat &img)
{
    Logger(YELLOW, "Adding Line... %d %d %d %d", line.start_point_x, line.start_point_y, line.end_point_x, line.end_point_y);
    rectangle(img, Point(Cm2PxY(line.start_point_y), Cm2PxX(line.start_point_x)), Point(Cm2PxY(line.end_point_y), Cm2PxX(line.end_point_x)), line.base_color, 5);
}

void AddGoalPost(const GoalPost &goal_post, Mat &img)
{
    Logger(YELLOW, "Adding GoalPost... %d %d %d %d", goal_post.start_point_x, goal_post.start_point_y, goal_post.end_point_x, goal_post.end_point_y);
    rectangle(img, Point(Cm2PxY(goal_post.start_point_y), Cm2PxX(goal_post.start_point_x)), Point(Cm2PxY(goal_post.end_point_y), Cm2PxX(goal_post.end_point_x)), CVWHITE, 5);
}

void AddText(const std::string &text, Mat &img, int x, int y)
{
    putText(img, text, Point(x, y), FONT_HERSHEY_SIMPLEX, 1, CVRED, 2);
}

void AddMiddleLine(const Field &field, const Line &line_, Mat &img)
{
    line(img, Point(field.length_y / 2, line_.start_point_x), Point(field.length_y / 2, line_.end_point_x), CVWHITE, 5);
}

void AddMiddleCircle(const Line &line_, Mat &img, int radius = 130)
{
    circle(img, Point((line_.start_point_y + line_.end_point_y) / 2, (line_.start_point_x + line_.end_point_x) / 2), radius, CVWHITE, 5);
}

void DrawRobot(const Robot &robot, Mat &img)
{
    Logger(YELLOW, "Draw robot on: %f %f %f", robot.pose.x, robot.pose.y, robot.pose.th);
    Logger(YELLOW, "Draw robot on: %f %f %f", Cm2PxX(robot.pose.x), Cm2PxY(robot.pose.y), robot.pose.th);
    cv::circle(img, cv::Point2f(Cm2PxY(robot.pose.y), Cm2PxX(robot.pose.x)), 30, CVROBOT, 2);
    cv::line(img, Point(Cm2PxY(robot.pose.y), Cm2PxX(robot.pose.x)), Point(Cm2PxX(robot.pose.x + 35 * sin(robot.pose.th * DEG2RAD)), Cm2PxY(robot.pose.y + 35 * cos(robot.pose.th * DEG2RAD))), CVROBOT, 3);
}

#endif