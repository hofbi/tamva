cmake_minimum_required(VERSION 3.0)
project(view_adaptation_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg FILES ViewPriority.msg Bandwidth.msg)

generate_messages(DEPENDENCIES view_adaptation_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
