cmake_minimum_required(VERSION 2.8)

# needed for aruco on car, since its is in /opt...
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(../hsogserver/)

set(RUNTIME_FILTER_NAME taco_RuntimeService)
adtf_add_filter(${RUNTIME_FILTER_NAME}
        decoder/ADTFPinMessageDecoder.h
        decoder/ADTFPinMessageDecoder.cpp
        decoder/InputPin.h
        decoder/Decoder.h
        decoder/Decoder.cpp
        decoder/GyroDecoder.h
        decoder/GyroDecoder.cpp
        decoder/AccelerometerDecoder.h
        decoder/AccelerometerDecoder.cpp
        decoder/IMUDecoder.h
        decoder/IMUDecoder.cpp
        decoder/ValueGroupDecoder.h
        decoder/ValueGroupDecoder.cpp
        decoder/DepthImageDecoder.h
        decoder/DepthImageDecoder.cpp
        decoder/CameraDecoder.h
        decoder/CameraDecoder.cpp
        decoder/JuryDecoder.h
        decoder/JuryDecoder.cpp
        decoder/RoadSignDecoder.h
        decoder/RoadSignDecoder.cpp
        decoder/ManeuverListDecoder.h
        decoder/ManeuverListDecoder.cpp
        decoder/WheelTickDecoder.h
        decoder/WheelTickDecoder.cpp

        encoder/ADTFPinMessageEncoder.h
        encoder/ADTFPinMessageEncoder.cpp
        encoder/OutputPin.h

        filters/RuntimeService.h
        filters/RuntimeService.cpp
        )
target_link_libraries(${RUNTIME_FILTER_NAME} ${TACO_LIB} ${TACO_DETECTION})
adtf_install_plugin(${RUNTIME_FILTER_NAME} ${AADC_INSTALL_DIR})

set(RATE_LIMIT_FILTER_NAME taco_RateLimit)
adtf_add_filter(${RATE_LIMIT_FILTER_NAME}
        encoder/ADTFPinMessageEncoder.h
        encoder/ADTFPinMessageEncoder.cpp
        encoder/OutputPin.h
        filters/RateLimit.h
        filters/RateLimit.cpp
        )
target_link_libraries(${RATE_LIMIT_FILTER_NAME} ${TACO_LIB})
adtf_install_plugin(${RATE_LIMIT_FILTER_NAME} ${AADC_INSTALL_DIR})

add_subdirectory(filters/imageWriter)
if (${TACO_CONFIG} STREQUAL TACO_2017)
    add_subdirectory(filters/fisheyeUndistortionROI)
    add_subdirectory(filters/cameraImageRelay)
endif ()