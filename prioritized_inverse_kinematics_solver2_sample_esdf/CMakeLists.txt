cmake_minimum_required(VERSION 3.0.2)
project(prioritized_inverse_kinematics_solver2_sample_esdf)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  ik_constraint2_esdf
  prioritized_inverse_kinematics_solver2
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



add_library(CnoidPrioritizedInverseKinematicsSolver2SampleEsdfPlugin SHARED
  src/CnoidPrioritizedInverseKinematicsSolver2SampleEsdfPlugin.cpp
  src/sample1_4limb.cpp
)
target_link_libraries(CnoidPrioritizedInverseKinematicsSolver2SampleEsdfPlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidPrioritizedInverseKinematicsSolver2SampleEsdfPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
