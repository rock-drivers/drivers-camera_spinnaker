rock_library(camera_spinnaker
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(camera_spinnaker_bin Main.cpp
    DEPS camera_spinnaker)

