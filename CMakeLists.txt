cmake_minimum_required(VERSION 3.10)

project(rtlinux_app VERSION 1.0.0 LANGUAGES C)

# Collect all source files
add_executable(rtlinux_app 
    src/main.c
    src/steer.c
    src/comm.c
    src/FWS.c
    src/speedControl.c
    src/Torque_Vectoring.c
    src/Torque_Vectoring_data.c
)

# Include directories
target_include_directories(rtlinux_app PRIVATE 
    include
)

# Link libraries
target_link_libraries(rtlinux_app
    pthread
    usb-1.0
    m
)