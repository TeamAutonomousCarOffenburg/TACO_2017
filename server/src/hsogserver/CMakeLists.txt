cmake_minimum_required(VERSION 2.8)

set(TACO_LIB taco)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(../../include)

if(${TACO_CONFIG} STREQUAL TACO_2017)
    set(PCL_DIR /opt/pcl/share/pcl-1.8)
    set(OpenCV_DIR /opt/opencv/3.2.0/share/OpenCV)
elseif(${TACO_CONFIG} STREQUAL TACO_2016)
    set(OpenCV_DIR /opt/opencv/3.0.0/share/OpenCV)
endif()

add_library(${TACO_LIB}
        AADCCar.h

        utils/pcl/DepthImageToPointCloudConverter.h
        utils/pcl/utils.h

        utils/geometry/Angle.h
        utils/geometry/Angle.cpp
        utils/geometry/Circle2D.h
        utils/geometry/Circle2D.cpp
        utils/geometry/Geometry.h
        utils/geometry/Geometry.cpp
        utils/geometry/Line2D.h
        utils/geometry/Line2D.cpp
        utils/geometry/LineSegment.h
        utils/geometry/LineSegment.cpp
        utils/geometry/Path2D.h
        utils/geometry/Path2D.cpp
        utils/geometry/Pose2D.h
        utils/geometry/Polygon.h
        utils/geometry/Polygon.cpp
        utils/geometry/Pose2D.cpp
        utils/geometry/Pose3D.h
        utils/geometry/Pose3D.cpp
        utils/geometry/Cosine.h
        utils/geometry/Cosine.cpp
        utils/geometry/BoundingBox2D.h
        utils/geometry/BoundingBox2D.cpp
        utils/geometry/AlignedBoundingBox3D.h
        utils/geometry/AlignedBoundingBox3D.cpp
        utils/geometry/IManeuverGeometry.h
        utils/geometry/drive/Driveable.h
        utils/geometry/drive/Driveable.cpp
        utils/geometry/drive/DriveGeometry.h
        utils/geometry/drive/DriveGeometry.cpp
        utils/geometry/drive/DriveCalculator.h
        utils/geometry/drive/DriveCalculator.cpp

        meta/ICarMetaModel.h
        meta/ISensorConfig.h
        meta/ICameraConfig.h
        meta/IActuatorConfig.h
        meta/IServoDriveConfig.h
        meta/IAxleConfig.h
        meta/IDistanceSensorConfig.h

        meta/impl/CarMetaModel.h
        meta/impl/CarMetaModel.cpp
        meta/impl/SensorConfig.h
        meta/impl/SensorConfig.cpp
        meta/impl/CameraConfig.h
        meta/impl/CameraConfig.cpp
        meta/impl/ActuatorConfig.h
        meta/impl/ActuatorConfig.cpp
        meta/impl/ServoDriveConfig.h
        meta/impl/ServoDriveConfig.cpp
        meta/impl/AxleConfig.h
        meta/impl/AxleConfig.cpp
        meta/impl/DistanceSensorConfig.h
        meta/impl/DistanceSensorConfig.cpp
        meta/impl/JSONCarMetaModel.h
        meta/impl/JSONCarMetaModel.cpp
        meta/impl/JSONCarMetaModelHandler.h

        meta/impl/ConfigFactories/IConfigFactory.h
        meta/impl/ConfigFactories/SensorConfigFactory.h
        meta/impl/ConfigFactories/DistanceSensorConfigFactory.h
        meta/impl/ConfigFactories/ConfigFactory.h
        meta/impl/ConfigFactories/Vector3DFactory.h
        meta/impl/ConfigFactories/AngleAxisdFactory.h
        meta/impl/ConfigFactories/CameraConfigFactory.h
        meta/impl/ConfigFactories/AxleConfigFactory.h
        meta/impl/ConfigFactories/ActuatorConfigFactory.h

        perception/IPerception.h
        perception/IPerceptor.h
        perception/IValuePerceptor.h
        perception/IGyroPerceptor.h
        perception/IIMUPerceptor.h
        perception/IJuryConstants.h
        perception/IJuryPerceptor.h
        perception/IRoadSignPerceptor.h
        perception/IManeuverListPerceptor.h
        perception/IAccelerometerPerceptor.h
        perception/ICameraPerceptor.h
        perception/IPointCloudPerceptor.h
        perception/IWheelTickPerceptor.h

        perception/impl/Perception.h
        perception/impl/Perception.cpp
        perception/impl/Perceptor.h
        perception/impl/ValuePerceptor.h
        perception/impl/GyroPerceptor.h
        perception/impl/JuryPerceptor.h
        perception/impl/RoadSignPerceptor.h
        perception/impl/ManeuverListPerceptor.h
        perception/impl/AccelerometerPerceptor.h
        perception/impl/CameraPerceptor.h
        perception/impl/PointCloudPerceptor.h
        perception/impl/IMUPerceptor.h
        perception/impl/WheelTickPerceptor.h
        perception/impl/Maneuver.h

        action/IAction.h
        action/IEffector.h
        action/IValueEffector.h
        action/IManeuverStatusEffector.h
        action/IPositionEffector.h
        action/IRoadSignEffector.h
        action/IParkingSpaceEffector.h
        action/IObstacleEffector.h

        action/impl/Action.h
        action/impl/Action.cpp
        action/impl/Effector.h
        action/impl/ValueEffector.h
        action/impl/ManeuverStatusEffector.h
        action/impl/PositionEffector.h
        action/impl/RoadSignEffector.h
        action/impl/ParkingSpaceEffector.h
        action/impl/ObstacleEffector.h

        utils/logger/DummyEventLogger.h
        utils/logger/IEventLogger.h
        utils/logger/EventLogger.h
        utils/logger/EventLogger.cpp
        utils/logger/LogReplayer.h
        utils/logger/LogReplayer.cpp

        utils/logger/TimeTaker.cpp
        utils/logger/TimeTaker.h

        utils/driveinstructionreader/DriveInstructionReader.h
        utils/driveinstructionreader/DriveInstructionReader.cpp
        utils/configurationreader/ConfigurationReader.cpp
        utils/configurationreader/ConfigurationReader.h
        utils/configurationreader/EnvironmentConfiguration.h
        utils/configurationreader/ConfigRoadSign.h
        utils/configurationreader/ParkingSpace.h

        utils/value/FuzzyCompare.cpp
        utils/value/FuzzyCompare.h

        utils/concurrency/Runnable.h
        utils/concurrency/Worker.h
        utils/concurrency/BlockingQueue.h

        utils/roadsigns/ISignConstants.h

        general/ComponentFactory.h
        general/ComponentFactory.cpp
        general/IComponentFactory.h
        general/LogComponentFactory.h

        communication/ByteConverter.h
        communication/Communication.cpp
        communication/Communication.h
        communication/PackageDecoder.cpp
        communication/PackageDecoder.h
        communication/PackageEncoder.cpp
        communication/PackageEncoder.h

        communication/Encoder/IEncoder.h
        communication/Encoder/PerceptorEncoder.h
        communication/Encoder/DoubleValuePerceptorEncoder.h
        communication/Encoder/IMUPerceptorEncoder.h
        communication/Encoder/WheelTickPerceptorEncoder.h
        communication/Encoder/ManeuverListPerceptorEncoder.h
        communication/Encoder/JuryPerceptorEncoder.h
        communication/Encoder/RoadSignPerceptorEncoder.h
        communication/Encoder/LaneAssistEncoder.h
        communication/Encoder/EncoderUtilities.h
        communication/Encoder/ObjectDetectionEncoder.h
        communication/Encoder/FloorNormalEncoder.h

        communication/Handler/IHandler.h
        communication/Handler/EffectorHandler.h
        communication/Handler/BoolEffectorHandler.h
        communication/Handler/DoubleEffectorHandler.h
        communication/Handler/ManeuverStatusEffectorHandler.h
        communication/Handler/PositionEffectorHandler.h
        communication/Handler/RoadSignEffectorHandler.h
        communication/Handler/ParkingSpaceEffectorHandler.h
        communication/Handler/ObstacleEffectorHandler.h
		communication/Encoder/EnvironmentConfigEncoder.h)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${aruco_INCLUDE_DIR})
include_directories(./)

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(${TACO_LIB} ${OpenCV_LIBS})
target_link_libraries(${TACO_LIB} ${Boost_LIBRARIES})
target_link_libraries(${TACO_LIB} ${PCL_COMMON_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})
target_link_libraries(${TACO_LIB} ${TACO_DETECTION})
target_link_libraries(${TACO_LIB} ${aruco_LIBS})

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(detection)

if(ADTF_FOUND)
    adtf_install_library(${TACO_LIB} ${AADC_INSTALL_DIR_LIBS})
endif()
