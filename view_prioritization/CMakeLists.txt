cmake_minimum_required(VERSION 3.0)
project(view_prioritization)

find_package(catkin REQUIRED COMPONENTS roslaunch rospy carla_msgs
                                        view_adaptation_msgs)

catkin_python_setup()

catkin_package()

catkin_add_nosetests(test)

roslaunch_add_file_check(launch)
