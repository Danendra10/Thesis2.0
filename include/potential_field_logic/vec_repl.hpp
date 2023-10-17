/**
 * @file vec_repl.h
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

class VectorRepulsive
{
private:
    float _max_field_radius, _gain_field_gradient, _c;

public:
    VectorRepulsive();
    ~VectorRepulsive();

    void init(float max_field_radius, float gain_field_gradient);
    void update(float x1, float y1, float x2, float y2, float &r, float &theta);
};

//------------------------------------------------------------------------------
//==============================================================================

VectorRepulsive::VectorRepulsive()
{
}

VectorRepulsive::~VectorRepulsive()
{
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * Initializes the VectorRepulsive object.
 *
 * @param max_field_radius the maximum field radius
 * @param gain_field_gradient the gain field gradient
 *
 * @throws ErrorType a description of the error, if any
 */
void VectorRepulsive::init(float max_field_radius, float gain_field_gradient)
{
    try
    {
        _max_field_radius = max_field_radius;
        _gain_field_gradient = gain_field_gradient * 400000;
    }
    catch (const std::exception &e)
    {
        Logger(RED, "Error on vec repulsive: %s", e.what());
    }
}

/**
 * Updates the vector repulsive based on the given coordinates.
 *
 * @param x1 the x-coordinate of the first point
 * @param y1 the y-coordinate of the first point
 * @param x2 the x-coordinate of the second point
 * @param y2 the y-coordinate of the second point
 * @param r the reference to the repulsive force
 * @param theta the reference to the angle between the points
 *
 * @throws None
 */
void VectorRepulsive::update(float x1, float y1, float x2, float y2, float &r, float &theta)
{
    float distance_between_points = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    float angle_between_points = atan2(y2 - y1, x2 - x1);
    if (distance_between_points < _max_field_radius && distance_between_points > 20)
        r = -_gain_field_gradient * (1 / pow(distance_between_points, 3) - ((1 / pow(distance_between_points, 2)) * (1 / _max_field_radius)));
    else
        r = 0;
    theta = angle_between_points;
}