# Require a minimum version of CMake
cmake_minimum_required(VERSION 3.16)

# Pull in the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(dashboard_node)
