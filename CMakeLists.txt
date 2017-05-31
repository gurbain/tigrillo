# Create a simulation library and executable to start a simulation of the Tigrillo quadruped
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(Tigrillo)

# Create executable
ADD_EXECUTABLE(simulation src/simulation/Simulation.cpp)

# Add link to Bullet Physics libraries
find_package (Bullet)
if (BULLET_FOUND)
  message(STATUS "Linking with the following Bullet Physics libraries: ${BULLET_LIBRARIES}")
  include_directories(${BULLET_INCLUDE_DIRS})
  target_link_libraries (simulation ${BULLET_LIBRARIES})
endif (BULLET_FOUND)
