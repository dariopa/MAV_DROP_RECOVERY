cmake_minimum_required(VERSION 2.8.3)
project(mav_drop_recovery)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

############
# BINARIES #
############
cs_add_executable(mav_drop_recovery
        src/planner.cc
        src/planner_node.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
