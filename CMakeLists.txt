cmake_minimum_required(VERSION 3.27)
project(THESIS)

add_compile_options(-std=c++17)

add_executable(THESIS src/main.cpp)
add_executable(2DVISUALIZATION src/potential_field_visualization_2d.cpp)
add_executable(3DVISUALIZATION src/potential_field_visualization_3d.cpp)
add_executable(OLD_METHOD src/old_method.cpp)
add_executable(repl_only src/repl_only.cpp)
add_executable(try_viz src/try_viz.cpp)
add_executable(robot_movement src/robot_movement.cpp)
add_executable(keyboard_event src/keyboard_event.cpp)

find_package(OpenCV REQUIRED)

include_directories(include)

IF(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(THESIS ${OpenCV_LIBS})
    target_link_libraries(2DVISUALIZATION ${OpenCV_LIBS})
    target_link_libraries(3DVISUALIZATION ${OpenCV_LIBS})
    target_link_libraries(OLD_METHOD ${OpenCV_LIBS})
    target_link_libraries(repl_only ${OpenCV_LIBS})
    target_link_libraries(try_viz ${OpenCV_LIBS})
    target_link_libraries(robot_movement ${OpenCV_LIBS})
    target_link_libraries(keyboard_event ${OpenCV_LIBS})
ENDIF(OpenCV_FOUND)