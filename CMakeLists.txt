cmake_minimum_required(VERSION 3.0.2)
project(my_nodes_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  )

find_package(Armadillo REQUIRED)

catkin_package( CATKIN_DEPENDS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
)

add_library( my_lib
 src/evaluation.cpp
 src/rewrite_imu.cpp
 src/get_imu_cov.cpp
 src/transform_ekf.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

add_executable(evaluation_node src/evaluation_node.cpp)
target_link_libraries(evaluation_node ${catkin_LIBRARIES} my_lib ${ARMADILLO_LIBRARIES})

add_executable(rewrite_imu_node src/rewrite_imu_node.cpp)
target_link_libraries(rewrite_imu_node my_lib ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(get_imu_cov_node src/get_imu_cov_node.cpp)
target_link_libraries(get_imu_cov_node my_lib ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(transform_ekf_node src/transform_ekf_node.cpp)
target_link_libraries(transform_ekf_node my_lib ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(pub_sim_dur_node src/pub_sim_dur_node.cpp)
target_link_libraries(pub_sim_dur_node ${catkin_LIBRARIES})
