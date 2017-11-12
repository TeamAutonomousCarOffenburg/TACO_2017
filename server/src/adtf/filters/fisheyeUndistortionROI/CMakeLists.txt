cmake_minimum_required(VERSION 2.8.4)

set(FILTER_NAME taco_FisheyeUndistortionROI)

include_directories(${OpenCV_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME}
        stdafx.h
        FisheyeUndistortionROI.h
        FisheyeUndistortionROI.cpp
        )

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS} ${TACO_DETECTION})
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
