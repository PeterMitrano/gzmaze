cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gzmaze)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -std=c++11")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(maze_plugin SHARED maze_plugin/MazePlugin.cc)
target_link_libraries(maze_plugin ${GAZEBO_LIBRARIES})

find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

QT4_WRAP_CPP(headers_MOC gui_plugin/RegenerateWidget.hh)

add_library(regenerate_widget SHARED ${headers_MOC}
  gui_plugin/RegenerateWidget.cc)
target_link_libraries(regenerate_widget
    ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(MousePlugin SHARED mouse_plugin/MousePlugin.cc)
target_link_libraries(MousePlugin	 ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
