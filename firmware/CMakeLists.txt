cmake_minimum_required(VERSION 3.16)
# Reuse the host-tested safety core as an IDF component.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firing_box)
