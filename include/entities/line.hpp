#ifndef LINE_HPP_
#define LINE_HPP_

#include "opencv2/opencv.hpp"

using namespace cv;

typedef struct Line
{
    Scalar base_color = Scalar(255, 255, 255);
    int start_point_x;
    int start_point_y;
    int end_point_x;
    int end_point_y;
} LineType;

#endif