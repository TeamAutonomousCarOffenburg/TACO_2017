cmake_minimum_required(VERSION 2.8.4)

include_directories(${OpenCV_INCLUDE_DIR})

set(FILTER_NAME taco_CameraImageRelay)

adtf_add_filter(${FILTER_NAME}
        CameraImageRelay.h
        CameraImageRelay.cpp
)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
