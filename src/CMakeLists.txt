#find_package(Spinnaker REQUIRED) #TO-DO: it seems spinanker library does not install a cmake find either pkg config

# Your installation should be here
set(Spinnaker_INCLUDE_DIR "/opt/spinnaker/include" CACHE PATH "Spinnaker Include Directory")
set(Spinnaker_LIBRARIES "/opt/spinnaker/lib/libSpinnaker.so")

message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIR}")
message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")

include_directories(${SPINNAKER_INCLUDE_DIR})

rock_library(camera_spinnaker
    SOURCES
        camera.cpp
        cm3.cpp
        gh3.cpp
        SpinnakerCamera.cpp
    HEADERS
        camera.h
        cm3.h
        gh3.h
        set_property.h
        SpinnakerCamera.h
        SpinnakerConfig.h
    DEPS_PLAIN
        Spinnaker
    DEPS_PKGCONFIG
        base-types
        base-logging
        frame_helper
        )

#rock_executable(camera_spinnaker_bin Main.cpp
#    DEPS camera_spinnaker)

