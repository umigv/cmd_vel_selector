cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(cmd_vel_selector)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp)

# 7.3 Include paths and library paths
# 7.3.1
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# 7.4 Executable targets
# $PROJECT_NAME set by project() in step 2 above
add_executable(${PROJECT_NAME}_node src/cmd_vel_selector_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node white_line_layer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
