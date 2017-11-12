cmake_minimum_required(VERSION 2.8)

find_package(GTest)

if(GTEST_FOUND)
	find_package(OpenCV REQUIRED)

	include_directories(${Boost_INCLUDE_DIRS})
	include_directories(${PCL_INCLUDE_DIRS})
	include_directories(${OpenCV_INCLUDE_DIR})

	add_executable(alltests LaneAssistTest.cpp)

	target_link_libraries(alltests gtest gtest_main)
	add_test(AllTestsInFoo alltests)

	target_link_libraries(alltests ${TACO_DETECTION})
	target_link_libraries(alltests ${TACO_LIB})
	target_link_libraries(alltests ${OpenCV_LIBS})
	target_link_libraries(alltests ${PCL_COMMON_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})
endif()


