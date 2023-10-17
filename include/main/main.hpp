#ifndef MAIN_HPP_
#define MAIN_HPP_
#define YAML_CPP_STATIC_DEFINE

#define FIELD_X_LENGTH 800
#define FIELD_Y_LENGTH 1200

#pragma once
// #include "robot.hpp"
#include "potential_field_logic/vec_attr.hpp"
#include "potential_field_logic/vec_repl.hpp"
#include "entities/draw.hpp"

// #include "yaml-cpp/yaml.h"
#include "opencv2/opencv.hpp"
#include "opencv2/viz.hpp"

#include <iostream>
#include <filesystem>
#include <fstream>

int max_attr_field_radius;
int max_repl_field_radius;

Mat kri_field = Mat::zeros(FIELD_X_LENGTH + 100, FIELD_Y_LENGTH + 100, CV_8UC3);

FieldType field;
LineType line;
GoalPostType goal_post_away;
GoalPostType goal_post_home;
RobotType robot_1_own(0, 0, 90, 0, 0, 0, OWN_ROBOT);
RobotType enemy_robot_1(100, 100, 90, 0, 0, 0, ENEMY_ROBOT);

int DrawField(viz::Viz3d &window)
{
    // Corners
    Point3d center(700, 500, 0);

    // Define the size of the field
    Size2d size(1400, 1000); // Field dimensions

    // Define the normal to the plane
    Vec3d normal(0, 0, 1);

    // Define the new y-axis for the plane
    Vec3d yaxis(0, 1, 0);

    // Create the green field using WPlane widget
    viz::WPlane field(center, normal, yaxis, size, viz::Color::green());

    // Display the field in the given window
    window.showWidget("Green Field", field);

    return 0;
}

int DrawOuterLine(viz::Viz3d &window)
{
    // outer line should be 50 cm from the field edge
    const uint8_t outer_line_offset = 50;

    Point3d p1(250 - outer_line_offset, 350 - outer_line_offset, 0);
    Point3d p2(1250 + outer_line_offset, 350 - outer_line_offset, 0);
    Point3d p3(1250 + outer_line_offset, 1050 + outer_line_offset, 0);
    Point3d p4(250 - outer_line_offset, 1050 + outer_line_offset, 0);

    std::vector<Point3d> outerLinePoints = {p1, p2, p3, p4, p1}; // Close the loop by adding p1 again

    viz::WPolyLine outerLine(outerLinePoints, viz::Color::white());

    window.showWidget("Outer Line", outerLine);

    return 0;
}

int DrawBox(viz::Viz3d &window, Point3d center, string widget_name, const viz::Color &color = viz::Color::magenta())
{
    Point3d size(20, 20, 20);
    // The bottom of the box will be at the center and the top will be center + size in the Z direction
    cv::Point3d bottom(center.x, center.y, center.z);
    cv::Point3d top(center.x + size.x, center.y + size.y, center.z + size.z);

    cv::viz::WCube robot(bottom, top, false, color);
    window.showWidget(widget_name, robot);

    return 0;
}

int PotentialFieldInit();
int PotentialFieldRoutine();
int LoadConfig();
int clearLogFile();

#endif