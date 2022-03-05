cmake_minimum_required(VERSION 3.0)
project(view_adaptation)

find_package(catkin REQUIRED COMPONENTS roslaunch)

roslaunch_add_file_check(launch)
