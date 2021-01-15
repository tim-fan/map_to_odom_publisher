cmake_minimum_required(VERSION 3.0.2)
project(map_to_odom_publisher)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            geometry_msgs
            rosconsole
            roscpp
            tf2_geometry_msgs
            tf2_ros
        )

find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        roscpp
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(map_to_odom_publisher src/map_to_odom_publisher.cpp)
target_link_libraries(map_to_odom_publisher
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
add_dependencies(map_to_odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
        map_to_odom_publisher
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)