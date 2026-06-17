cmake_minimum_required(VERSION 3.16)

# Pull the parent directory in as an extra component so the example can
# reference `ps2keyboard` without having to be installed via the Component
# Manager.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ps2keyboard_simple_test)
