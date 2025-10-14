cmake_minimum_required(VERSION 3.10)
project(ankle_exo_torque_profile C CXX)

# -----------------------------------------------------------------------------
# Build options
# -----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# -----------------------------------------------------------------------------
# ADS1115 paths (update if different)
# -----------------------------------------------------------------------------
set(ADS1115_DIR /home/ankleexo/ads1115)

# -----------------------------------------------------------------------------
# Include directories
# -----------------------------------------------------------------------------
include_directories(
    include
    ${CMAKE_SOURCE_DIR}/serialPort
    ${CMAKE_SOURCE_DIR}/unitreeMotor
    ${ADS1115_DIR}/src
    ${ADS1115_DIR}/interface
    ${ADS1115_DIR}/project/raspberrypi4b/interface/inc
)

# -----------------------------------------------------------------------------
# Library directories
# -----------------------------------------------------------------------------
link_directories(
    lib
)

# -----------------------------------------------------------------------------
# Detect platform and select proper Unitree SDK library
# -----------------------------------------------------------------------------
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(EXTRA_LIBS libUnitreeMotorSDK_Arm64.so)
else()
    set(EXTRA_LIBS libUnitreeMotorSDK_Linux64.so)
endif()

# -----------------------------------------------------------------------------
# Real-time dependencies (Threads + librt)
# -----------------------------------------------------------------------------
find_package(Threads REQUIRED)
find_library(RT_LIB rt)

add_compile_definitions(_GNU_SOURCE _POSIX_C_SOURCE=200809L)

# Ensure runtime can find .so in ./lib
set(CMAKE_BUILD_RPATH "${CMAKE_SOURCE_DIR}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# -----------------------------------------------------------------------------
# Source files
# -----------------------------------------------------------------------------
add_executable(ankle_exo_torque_profile
    src/main.cpp
    ${ADS1115_DIR}/src/driver_ads1115.c
    ${ADS1115_DIR}/project/raspberrypi4b/driver/src/raspberrypi4b_driver_ads1115_interface.c
    ${ADS1115_DIR}/project/raspberrypi4b/interface/src/iic.c
)

# -----------------------------------------------------------------------------
# Link dependencies
# -----------------------------------------------------------------------------
if(RT_LIB)
    target_link_libraries(ankle_exo_torque_profile PRIVATE
        ${EXTRA_LIBS}
        Threads::Threads
        ${RT_LIB}
        i2c
        pthread
    )
else()
    target_link_libraries(ankle_exo_torque_profile PRIVATE
        ${EXTRA_LIBS}
        Threads::Threads
        i2c
        pthread
    )
endif()

# -----------------------------------------------------------------------------
# Optional: grant CAP_SYS_NICE (so SCHED_FIFO works without sudo)
# -----------------------------------------------------------------------------
option(GRANT_RT_CAP "Add helper target to set CAP_SYS_NICE on RT binaries" ON)
if(GRANT_RT_CAP)
  add_custom_target(grant_caps
    COMMAND ${CMAKE_COMMAND} -E echo "Granting CAP_SYS_NICE to ankle_exo_torque_profile (requires sudo)..."
    COMMAND sudo setcap cap_sys_nice+ep $<TARGET_FILE:ankle_exo_torque_profile>
    VERBATIM
    COMMENT "setcap cap_sys_nice+ep"
  )
endif()