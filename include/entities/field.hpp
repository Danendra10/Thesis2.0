#ifndef FIELD_HPP_
#define FIELD_HPP_

#include <vector>
#include "opencv2/opencv.hpp"

using namespace cv;

typedef struct Field
{
    /* data */
    Scalar base_color = Scalar(0, 255, 0);
    int length_x;
    int length_y;
} FieldType;

#endif