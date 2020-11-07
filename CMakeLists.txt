cmake_minimum_required(VERSION 2.8)
project(gzmaze)

set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(gazebo REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(regenerate_widget SHARED gui_plugin/RegenerateWidget.h gui_plugin/RegenerateWidget.cc)
target_include_directories(regenerate_widget PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(regenerate_widget Qt5::Widgets Qt5::Core ${GAZEBO_LIBRARIES})

add_library(maze_plugin SHARED maze_plugin/MazePlugin.cc)
target_include_directories(maze_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(maze_plugin ${GAZEBO_LIBRARIES})
target_link_directories(maze_plugin PUBLIC ${GAZEBO_LIBRARY_DIRS})

add_library(MousePlugin SHARED mouse_plugin/MousePlugin.cc)
target_include_directories(MousePlugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(MousePlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
