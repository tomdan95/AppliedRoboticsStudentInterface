cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(project_interface REQUIRED)

## Specify additional locations of header files
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${project_interface_INCLUDE_DIRS}
)

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
        src/student_interface.cpp
        src/pipeline/image_undistort.cpp
        src/pipeline/extrinsic_calibration.cpp
        src/pipeline/unwrap.cpp
        src/pipeline/detection/find_robot.cpp
        src/pipeline/detection/process_map.cpp
        src/pipeline/planning/planning.cpp
        src/pipeline/planning/dubins/curve.cpp
        src/pipeline/planning/dubins/curve.h
        src/pipeline/planning/dubins/primitives.cpp
        src/pipeline/planning/dubins/primitives.h
        src/pipeline/planning/dubins/scale.cpp
        src/pipeline/planning/dubins/scale.h
        src/pipeline/planning/dubins/temp.cpp
        src/pipeline/planning/dubins/temp.h
        src/pipeline/planning/dubins/utils.cpp
        src/pipeline/planning/dubins/utils.h
        src/pipeline/planning/dubins/dubins.cpp
        src/pipeline/planning/dubins/dubins.h)

target_link_libraries(student
        ${OpenCV_LIBRARIES}
        )

## COMPILE DEMO EXECUTABLE
add_executable(demo_dilatation_erosion
        src/demo_shape_detection/demo_dilatation_erosion.cpp
        )
add_executable(demo_hsv_filter
        src/demo_shape_detection/demo_hsv_filter.cpp
        )
add_executable(demo_smoothing
        src/demo_shape_detection/demo_smoothing.cpp
        )
add_executable(demo_filters
        src/demo_shape_detection/demo_filters.cpp
        )
add_executable(demo_rgb_filter
        src/demo_shape_detection/demo_rgb_filter.cpp
        )
add_executable(demo_full_example
        src/demo_shape_detection/demo_full_example.cpp
        )
add_executable(demo_simple_shape_detection
        src/demo_shape_detection/demo_simple_shape_detection.cpp
        )

target_link_libraries(demo_dilatation_erosion
        ${OpenCV_LIBRARIES}
        )
target_link_libraries(demo_hsv_filter
        ${OpenCV_LIBRARIES}
        )
target_link_libraries(demo_smoothing
        ${OpenCV_LIBRARIES}
        )
target_link_libraries(demo_filters
        ${OpenCV_LIBRARIES}
        )
target_link_libraries(demo_rgb_filter
        ${OpenCV_LIBRARIES}
        )
target_link_libraries(demo_full_example
        ${OpenCV_LIBRARIES}
        )
target_link_libraries(demo_simple_shape_detection
        ${OpenCV_LIBRARIES}
        )

## Add match template executable
add_executable(demo_match_template
        src/match_template.cpp
        )
# Link with libraries
target_link_libraries(demo_match_template
        ${OpenCV_LIBRARIES}
        )


add_executable(dubins_main
        src/pipeline/planning/dubins/curve.cpp
        src/pipeline/planning/dubins/curve.h
        src/pipeline/planning/dubins/primitives.cpp
        src/pipeline/planning/dubins/primitives.h
        src/pipeline/planning/dubins/scale.cpp
        src/pipeline/planning/dubins/scale.h
        src/pipeline/planning/dubins/temp.cpp
        src/pipeline/planning/dubins/temp.h
        src/pipeline/planning/dubins/utils.cpp
        src/pipeline/planning/dubins/utils.h
        src/pipeline/planning/dubins/dubins.cpp
        src/pipeline/planning/dubins/dubins.h
        src/pipeline/planning/dubins/main.cpp)
