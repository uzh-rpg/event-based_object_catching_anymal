cmake_minimum_required(VERSION 2.8.3)
project(rpg_dynamic_obstacle_detection_msgs)

find_package(vision_msgs REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(message_generation REQUIRED)

catkin_simple()

cs_install()
cs_export()
