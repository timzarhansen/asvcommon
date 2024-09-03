cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CATKIN_ENABLE_TESTING OFF)
project(asvcommon)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(tf2 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)
#find_package(Qt5 COMPONENTS Core Gui  Widgets Gamepad REQUIRED)
##find_package(OpenCV REQUIRED)
#find_package(Qt5PrintSupport REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(waterlinked_a50 REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(asvcommonmsg REQUIRED)
include_directories(src/generalTools)

add_executable(ekfNode src/stateEstimation/ekfDVLnode.cpp src/stateEstimation/pose.h src/stateEstimation/ekfDVL.cpp src/stateEstimation/ekfDVL.h src/generalTools/generalHelpfulTools.cpp)
ament_target_dependencies(ekfNode rclcpp std_msgs sensor_msgs waterlinked_a50 tf2 Eigen3 px4_msgs asvcommonmsg)
#target_link_libraries(ekfNode )
#
#
#
install(TARGETS
        ekfNode
#        controllerbluerov2
#        visualizationPing360
#        #        guibluerov2
        DESTINATION lib/${PROJECT_NAME}
)
#
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
#
#
#ament_python_install_package(${PROJECT_NAME})
#
#install(PROGRAMS
#        scripts/pwmServicesTopTube.py
#        scripts/pwmServicesSensorTube.py
#        scripts/leakageSensorTopTube.py
#        scripts/leakageSensorSensorTube.py
#        scripts/powerControlBotomTube.py
#        scripts/mjpeg_cam_python.py
#        bashScripts/xrcClientStart.sh
#        bashScripts/startpigpiod.sh
#        DESTINATION lib/${PROJECT_NAME}
#)



ament_package()