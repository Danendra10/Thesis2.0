cmake_minimum_required(VERSION 3.27)
project(THESIS)

add_compile_options(-std=c++17)

add_executable(THESIS src/main.cpp)

find_package(OpenCV REQUIRED)

include_directories(include)

IF(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(THESIS ${OpenCV_LIBS})
ENDIF(OpenCV_FOUND)

