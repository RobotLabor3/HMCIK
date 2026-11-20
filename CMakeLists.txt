cmake_minimum_required(VERSION 3.8)
project(hmcik)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  roscpp
  roslib
  sensor_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED CONFIG)
find_package(fcl REQUIRED CONFIG)
find_package(octomap REQUIRED CONFIG)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LeftArmKine RightArmKine
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs visualization_msgs
  DEPENDS EIGEN3
)

add_library(LeftArmKine SHARED src/left_arm_kine.cpp)
add_library(RightArmKine SHARED src/right_arm_kine.cpp)
foreach(target LeftArmKine RightArmKine)
  target_compile_features(${target} PUBLIC cxx_std_14)
  target_include_directories(${target} PUBLIC include ${catkin_INCLUDE_DIRS})
  target_link_libraries(${target}
    PUBLIC Eigen3::Eigen
    PRIVATE fcl octomap octomath ${catkin_LIBRARIES}
  )
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
endforeach()

add_executable(hmcik_test src/hmcik_test.cpp)
target_link_libraries(hmcik_test PRIVATE LeftArmKine RightArmKine ${catkin_LIBRARIES})
add_dependencies(hmcik_test ${catkin_EXPORTED_TARGETS})

install(TARGETS LeftArmKine RightArmKine hmcik_test
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY resource DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
