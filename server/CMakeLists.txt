cmake_minimum_required(VERSION 2.8.4)
project(taco-server)

# general settings
cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O0 -Wall -fPIC -Wno-literal-suffix -Wno-sign-compare")

# handle different car models
set(TACO_CONFIGS TACO_2016 TACO_2017)
if (NOT ";${TACO_CONFIGS};" MATCHES ";${TACO_CONFIG};")
    message(FATAL_ERROR "TACO_CONFIG must be set to one of the following: {${TACO_CONFIGS}}\nExample: -DTACO_CONFIG=TACO_2017")
endif()
message("Building version ${TACO_CONFIG}...")
add_definitions(-DTACO_2016=1 -DTACO_2017=2)

if(${TACO_CONFIG} STREQUAL TACO_2017)
    set(PCL_DIR /opt/pcl/share/pcl-1.8)
    add_definitions(-DTACO_CONFIG=2)

    set(ADTF_DIR              /opt/adtf/2.14.0)
    set(AADC_DIR              /home/aadc/ADTF)
    set(AADC_INSTALL_DIR      /home/aadc/ADTF/_install/linux64/bin)
    set(AADC_INSTALL_DIR_LIBS /home/aadc/ADTF/_install/linux64/lib)

elseif(${TACO_CONFIG} STREQUAL TACO_2016)
    add_definitions(-DTACO_CONFIG=1)

    set(ADTF_DIR              /opt/adtf/2.13.1)
    set(AADC_DIR              /home/aadc/repos/aadc/AADC/src)
    set(AADC_INSTALL_DIR      /home/aadc/repos/aadc/AADC/bin/Linux/gcc)
    set(AADC_INSTALL_DIR_LIBS /home/aadc/repos/aadc/AADC/bin/Linux/gcc)
endif()

find_package(aruco REQUIRED PATHS "/opt/aruco/1.3.0/lib" CONFIGS Findaruco.cmake)
find_package(ADTF REQUIRED)
find_package(AADC REQUIRED)

include_directories(include)
include_directories(${AADC_DIR}/include)

set(TACO_LIB taco)
set(TACO_DETECTION detection)

add_subdirectory(src)