# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.9)
project(camera_spinnaker
        VERSION 0.1
        DESCRIPTION "Camera Spinnaker using the FLIR Spinnaker SDK")
find_package(Rock)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ROCK_TEST_ENABLED ON)

rock_init()
rock_standard_layout()
