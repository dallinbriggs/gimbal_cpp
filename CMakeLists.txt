cmake_minimum_required(VERSION 2.8.3)
project(gimbal_serializer)
set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    )

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES async_comm
    CATKIN_DEPENDS roscpp
    DEPENDS Boost)

include_directories(
    include
    async_comm/include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_library(async_comm
    async_comm/src/serial.cpp
    async_comm/src/comm.cpp)

add_executable(gimbal_serializer
    src/gimbal_serializer.cpp
    )
add_dependencies(gimbal_serializer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gimbal_serializer
    ${catkin_LIBRARIES}
    ${BOOST_LIBRARIES}
    async_comm
    )


# install(TARGETS gimbal_serializer gimbal_serializer)
