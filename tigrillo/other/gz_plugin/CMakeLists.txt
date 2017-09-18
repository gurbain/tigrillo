cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(tigrillo_plugin SHARED tigrillo_plugin.cc)
target_link_libraries(tigrillo_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

# Build the stand-alone test program
#add_executable(test_tigrillo tigrillo_plugin_test.cc)

# if (${gazebo_VERSION_MAJOR} LESS 6)
#   include(FindBoost)
#   find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#   target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# else()
#   target_link_libraries(vel ${GAZEBO_LIBRARIES})
# endif()