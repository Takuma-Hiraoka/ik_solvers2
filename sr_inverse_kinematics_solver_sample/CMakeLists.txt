cmake_minimum_required(VERSION 3.0.2)
project(sr_inverse_kinematics_solver_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  sr_inverse_kinematics_solver
  roslib
  choreonoid_viewer
  )
find_package(choreonoid REQUIRED)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidSRInverseKinematicsSolverSamplePlugin SHARED
  src/CnoidSRInverseKinematicsSolverSamplePlugin.cpp
  src/sample1.cpp
)
target_link_libraries(CnoidSRInverseKinematicsSolverSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidSRInverseKinematicsSolverSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
