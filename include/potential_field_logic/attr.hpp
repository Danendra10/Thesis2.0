/**
 * @file vec_attr.h
 * @author Danendra (danendraclever24@gmail.com)
 * @Danendra10
 * @brief
 * @version 0.1
 * @date 2023-03-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "math.h"
#include "logger/logger.hpp"
#include <exception>

class VectorAttractive
{
private:
    float _max_field_radius, _max_field_gradient;

    // Function 1 => y = ax + b
    // Function 2 => y = cx^2
    float _a, _b, _c;

public:
    VectorAttractive();
    ~VectorAttractive();

    void init(float max_field_radius, float max_field_gradient);
    void update(float x1, float y1, float x2, float y2, float &r, float &theta);
};

//------------------------------------------------------------------------------
//==============================================================================

VectorAttractive::VectorAttractive()
{
}

VectorAttractive::~VectorAttractive()
{
}

//------------------------------------------------------------------------------
//==============================================================================

void VectorAttractive::init(float max_field_radius, float max_field_gradient)
{
    try
    {
        _max_field_radius = max_field_radius;
        _max_field_gradient = max_field_gradient;

        _c = _max_field_gradient / (2 * _max_field_radius);
        _a = _max_field_gradient;
        _b = _c * _max_field_radius * _max_field_radius - _max_field_radius;
    }
    catch (const std::exception &e)
    {
        Logger(RED, "Error on vec attractive: %s", e.what());
    }
}

/**
 * Updates the vector attractive based on the given coordinates.
 *
 * @param x1 The x-coordinate of the first point.
 * @param y1 The y-coordinate of the first point.
 * @param x2 The x-coordinate of the second point.
 * @param y2 The y-coordinate of the second point.
 * @param r The updated value of the radius.
 * @param theta The updated value of the angle.
 */
void VectorAttractive::update(float x1, float y1, float x2, float y2, float &r, float &theta)
{
    float distance_between_points = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    float angle_between_points = atan2(y2 - y1, x2 - x1);

    if (distance_between_points <= _max_field_radius)
        r = 2 * _c * distance_between_points;
    else
        r = _a;

    theta = angle_between_points;
}
