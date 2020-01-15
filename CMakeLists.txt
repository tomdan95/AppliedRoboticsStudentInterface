cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(project_interface REQUIRED)

## Specify additional locations of header files
include_directories(
        include
        libs
        ${OpenCV_INCLUDE_DIRS}
        ${project_interface_INCLUDE_DIRS}
)

###### START OF STRUDENT LIBRARY #####
add_library(student SHARED
        libs/clipper/clipper.hpp
        libs/clipper/clipper.cpp
        src/student_interface.cpp
        src/pipeline/image_undistort.cpp
        src/pipeline/extrinsic_calibration.cpp
        src/pipeline/unwrap.cpp
        src/pipeline/detection/find_robot.cpp
        src/pipeline/detection/process_map.cpp
        src/pipeline/planning/planning.cpp
        src/pipeline/planning/dubins/primitives.cpp
        src/pipeline/planning/dubins/primitives.h
        src/pipeline/planning/dubins/utils.cpp
        src/pipeline/planning/dubins/utils.h
        src/pipeline/planning/dubins/dubins.cpp
        src/pipeline/planning/dubins/dubins.h
        src/pipeline/planning/Graph.cpp
        src/pipeline/planning/Graph.h
        src/pipeline/detection/find_robot.hpp
        src/pipeline/detection/ShapeDetector.cpp
        src/pipeline/detection/ShapeDetector.h
        src/opencv-utils.cpp
        src/opencv-utils.h
        src/pipeline/detection/find_victims.cpp
        src/pipeline/detection/find_victims.hpp
        src/pipeline/detection/find_obstacles.hpp
        src/pipeline/detection/find_obstacles.cpp
        src/pipeline/detection/find_gate.hpp
        src/pipeline/detection/find_gate.cpp
        src/pipeline/detection/digit_classification/DigitClassifier.cpp
        src/pipeline/detection/digit_classification/DigitClassifier.h
        src/pipeline/planning/planning.h
        src/pipeline/planning/inflate.cpp
        src/pipeline/utils.cpp
        src/pipeline/utils.h
        src/pipeline/planning/voronoi/voronoi_cleanest_path.cpp
        src/pipeline/planning/voronoi/voronoi_cleanest_path.h
        src/pipeline/planning/collision_detection/ShadowCollisionDetector.cpp
        src/pipeline/planning/collision_detection/ShadowCollisionDetector.h
        src/pipeline/DebugImage.cpp src/pipeline/DebugImage.h
        src/pipeline/planning/best_theta/BestThetaFinder.cpp
        src/pipeline/planning/best_theta/BestThetaFinder.h
        src/pipeline/planning/dubins/models.cpp
        src/pipeline/planning/dubins/models.h
        src/pipeline/planning/MissionSolver.h
        src/pipeline/planning/Mission1.cpp
        src/pipeline/planning/Mission1.h
        src/pipeline/planning/MissionSolver.cpp
        src/pipeline/planning/Mission2.cpp
        src/pipeline/planning/Mission2.h
        src/pipeline/planning/collision_detection/CollisionDetector.h
        src/pipeline/Config.cpp src/pipeline/Config.h src/pipeline/planning/Mission1RTT.cpp src/pipeline/planning/Mission1RTT.h)

target_link_libraries(student
        ${OpenCV_LIBRARIES}
        )

###### END OF STRUDENT LIBRARY #####
#
###### START OF demo_dilatation_erosion #####
###[[add_executable(demo_dilatation_erosion
##        src/demos/demo_shape_detection/demo_dilatation_erosion.cpp
##        )
##
##target_link_libraries(demo_dilatation_erosion
##        ${OpenCV_LIBRARIES}
##        )]]
###### END OF demo_dilatation_erosion #####
##
###### START OF demo_hsv_filter #####
add_executable(demo_hsv_filter
        src/demos/demo_shape_detection/demo_hsv_filter.cpp
        )

target_link_libraries(demo_hsv_filter
        ${OpenCV_LIBRARIES}
        )
###### START OF demo_hsv_filter #####
#
###### START OF demo_smoothing #####
#add_executable(demo_smoothing
#        src/demos/demo_shape_detection/demo_smoothing.cpp
#        )
#target_link_libraries(demo_smoothing
#        ${OpenCV_LIBRARIES}
#        )
###### END OF demo_smoothing #####
#
###### START OF demo_filters #####
#add_executable(demo_filters
#        src/demos/demo_shape_detection/demo_filters.cpp
#        )
#
#target_link_libraries(demo_filters
#        ${OpenCV_LIBRARIES}
#        )
#
###### END OF demo_filters #####
#
#
###### START OF demo_rgb_filter #####
#add_executable(demo_rgb_filter
#        src/demos/demo_shape_detection/demo_rgb_filter.cpp
#        )
#
#target_link_libraries(demo_rgb_filter
#        ${OpenCV_LIBRARIES}
#        )
###### END OF demo_rgb_filter #####
#
###### START OF demo_full_example #####
#add_executable(demo_full_example
#        src/demos/demo_shape_detection/demo_full_example.cpp
#        )
#
#target_link_libraries(demo_full_example
#        ${OpenCV_LIBRARIES}
#        )
#
###### END OF demo_full_example #####
#
###### START OF demo_simple_shape_detection #####
#add_executable(demo_simple_shape_detection
#        src/demos/demo_shape_detection/demo_simple_shape_detection.cpp
#        )
#target_link_libraries(demo_simple_shape_detection
#        ${OpenCV_LIBRARIES}
#        )
###### END OF demo_simple_shape_detection #####
#
###### START OF demo_match_template #####
#add_executable(demo_match_template
#        src/demos/match_template.cpp
#        )
#target_link_libraries(demo_match_template
#        ${OpenCV_LIBRARIES}
#        )
###### END OF demo_match_template #####
#
#
###### START OF duins_demo #####
#add_executable(dubins_demo
#        src/pipeline/planning/dubins/primitives.cpp
#        src/pipeline/planning/dubins/primitives.h
#        src/pipeline/planning/dubins/utils.cpp
#        src/pipeline/planning/dubins/utils.h
#        src/pipeline/planning/dubins/dubins.cpp
#        src/pipeline/planning/dubins/dubins.h
#        src/demos/debins.cpp src/pipeline/planning/dubins/models.cpp src/pipeline/planning/dubins/models.h)
###### END OF duins_demo #####
#
#
###### START of find_robot_demo ####
#add_executable(find_robot_demo
#        src/demos/find_robot.cpp
#        src/pipeline/detection/find_robot.cpp
#        src/pipeline/detection/find_robot.hpp
#        src/pipeline/detection/ShapeDetector.cpp
#        src/pipeline/detection/ShapeDetector.h
#        src/opencv-utils.cpp
#        src/opencv-utils.h
#        src/pipeline/utils.cpp
#        src/pipeline/utils.h)
#
#target_link_libraries(find_robot_demo
#        ${OpenCV_LIBRARIES}
#        )
###### END of find_robot_demo ####
#
###### START of find_victim_demo ####
#add_executable(find_victim_demo
#        src/demos/find_victims.cpp
#        src/pipeline/detection/find_victims.cpp
#        src/pipeline/detection/find_victims.hpp
#        src/pipeline/detection/ShapeDetector.cpp
#        src/pipeline/detection/ShapeDetector.h
#        src/pipeline/detection/digit_classification/DigitClassifier.cpp
#        src/pipeline/detection/digit_classification/DigitClassifier.h
#        src/pipeline/Config.cpp
#        src/pipeline/Config.h
#        src/opencv-utils.cpp
#        src/opencv-utils.h)
#
#target_link_libraries(find_victim_demo
#        ${OpenCV_LIBRARIES}
#        )
###### END of find_robot_demo ####
#
###### START of planning_demo ####
#add_executable(planning_demo
#        src/demos/planning.cpp
#        src/pipeline/planning/inflate.cpp
#        libs/clipper/clipper.hpp
#        libs/clipper/clipper.cpp
#        src/pipeline/unwrap.cpp
#        src/pipeline/detection/find_robot.cpp
#        src/pipeline/detection/process_map.cpp
#        src/pipeline/planning/planning.cpp
#        src/pipeline/planning/dubins/primitives.cpp
#        src/pipeline/planning/dubins/primitives.h
#        src/pipeline/planning/dubins/utils.cpp
#        src/pipeline/planning/dubins/utils.h
#        src/pipeline/planning/dubins/dubins.cpp
#        src/pipeline/planning/dubins/dubins.h
#        src/pipeline/planning/Graph.cpp
#        src/pipeline/planning/Graph.h
#        src/pipeline/detection/find_robot.hpp
#        src/pipeline/detection/ShapeDetector.cpp
#        src/pipeline/detection/ShapeDetector.h
#        src/opencv-utils.cpp
#        src/opencv-utils.h
#        src/pipeline/detection/find_victims.cpp
#        src/pipeline/detection/find_victims.hpp
#        src/pipeline/detection/find_obstacles.hpp
#        src/pipeline/detection/find_obstacles.cpp
#        src/pipeline/detection/find_gate.hpp
#        src/pipeline/detection/find_gate.cpp
#        src/pipeline/detection/digit_classification/DigitClassifier.cpp
#        src/pipeline/detection/digit_classification/DigitClassifier.h
#        src/pipeline/planning/planning.h
#        src/pipeline/utils.cpp
#        src/pipeline/utils.h
#        src/pipeline/planning/voronoi/voronoi_cleanest_path.cpp
#        src/pipeline/planning/voronoi/voronoi_cleanest_path.h
#        src/pipeline/planning/collision_detection/ShadowCollisionDetector.cpp
#        src/pipeline/planning/collision_detection/ShadowCollisionDetector.h
#        src/pipeline/DebugImage.cpp src/pipeline/DebugImage.h
#        src/pipeline/planning/best_theta/BestThetaFinder.cpp
#        src/pipeline/planning/best_theta/BestThetaFinder.h
#        src/pipeline/planning/dubins/models.cpp src/pipeline/planning/dubins/models.h
#        src/pipeline/planning/MissionSolver.h src/pipeline/planning/Mission1.cpp
#        src/pipeline/planning/Mission1.h src/pipeline/planning/MissionSolver.cpp src/pipeline/planning/Mission2.cpp src/pipeline/planning/Mission2.h src/pipeline/planning/collision_detection/CollisionDetector.h src/pipeline/Config.cpp src/pipeline/Config.h
#        src/opencv-utils.cpp
#        src/opencv-utils.h)
#
#target_link_libraries(planning_demo
#        ${OpenCV_LIBRARIES}
#        )
###### END of planning_demo ####
#
## WARNING: Cyclic DEPENDENCY!!!
###### START of collision_detector_demo ####
##add_executable(collision_detector_demo
##        src/demos/collision_detector.cpp
##        src/pipeline/planning/collision_detection/CollisionDetector.cpp
##        src/pipeline/planning/collision_detection/CollisionDetector.h
##        src/pipeline/planning/best_theta/best_theta.cpp src/pipeline/planning/best_theta/best_theta.h)
#
##target_link_libraries(collision_detector_demo
##        ${OpenCV_LIBRARIES}
##        )
###### END of planning_demo ####
#
###### START of best_theta_demo ####
#add_executable(best_theta_demo
#        src/demos/best_theta.cpp
#        src/pipeline/planning/best_theta/BestThetaFinder.cpp
#        src/pipeline/planning/dubins/primitives.cpp
#        src/pipeline/planning/dubins/primitives.h
#        src/pipeline/planning/dubins/utils.cpp
#        src/pipeline/planning/dubins/utils.h
#        src/pipeline/planning/dubins/dubins.cpp
#        src/pipeline/planning/dubins/dubins.h
#        src/pipeline/planning/Graph.cpp
#        src/pipeline/planning/Graph.h
#        src/pipeline/DebugImage.cpp
#        src/opencv-utils.cpp
#        src/pipeline/planning/collision_detection/ShadowCollisionDetector.cpp
#        src/pipeline/planning/collision_detection/ShadowCollisionDetector.h
#        src/pipeline/planning/dubins/models.cpp src/pipeline/planning/dubins/models.h src/pipeline/planning/collision_detection/CollisionDetector.h src/pipeline/Config.cpp src/pipeline/Config.h)
#
#target_link_libraries(best_theta_demo
#        ${OpenCV_LIBRARIES}
#        )
###### END of planning_demo ####
