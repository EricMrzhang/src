cmake_minimum_required(VERSION 3.5)
project(olelidar)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  laser_geometry
  message_generation
  roscpp
  roscpp_serialization
  roslint
  sensor_msgs
  std_msgs
  tf
  common
)

find_package(Boost REQUIRED COMPONENTS  thread system)
#find_package(jsoncpp REQUIRED)
#get_target_property(jsoncpp_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(CURLPP REQUIRED curlpp)

generate_dynamic_reconfigure_options(
  cfg/oleiPuck.cfg
)

add_message_files(
  FILES
  VFHeader.msg
  1FHeader.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp roscpp_serialization sensor_msgs std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_main src/ole_packet/ole_packet.cpp                   
                        src/ole_packet/VF_packet.cpp
                        src/ole_packet/VF_packet_a.cpp
                        src/ole_packet/VF_packet_b.cpp
                        src/ole_packet/VF_packet_c.cpp
                        src/communication/udp_transport.cpp
                        src/communication/tcp_transport.cpp
                        src/ros_main.cpp
                        src/ros_node.cpp
                        )
                        
target_link_libraries(ros_main
  ${catkin_LIBRARIES} curl
)
add_dependencies(ros_main ${PROJECT_NAME}_gencfg)
add_dependencies(ros_main ${${PROJECT_NAME}_EXPORTED_TARGETS})

roslint_cpp(src/ros_main.cpp
            src/ros_node.cpp
            src/ole_packet/ole_packet.cpp
            src/communication.cpp)

#if (CATKIN_ENABLE_TESTING)
#  set(${PROJECT_NAME}_TEST_SOURCES
#    tests/http_request.cpp
#    tests/pfsdp_protocol.cpp)

#  catkin_add_gtest(pf_driver_test ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_TEST_SOURCES} tests/main.cpp)
#  target_link_libraries(pf_driver_test ${catkin_LIBRARIES} cpprestsdk::cpprest)
#endif()

install(TARGETS ros_main ros_main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# install(DIRECTORY rviz/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
