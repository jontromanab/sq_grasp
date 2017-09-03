cmake_minimum_required(VERSION 2.8.3)
project(sq_grasping)

add_compile_options(-std=c++11)
add_compile_options(-std=gnu++11)
add_compile_options(-fext-numeric-literals)

find_package(catkin REQUIRED COMPONENTS
    sq_fitting
    grasp_execution
    cmake_modules
    roscpp
    message_generation
    geometry_msgs
    pcl_ros
    pcl_conversions
    sensor_msgs
    tf2)


find_package(Boost REQUIRED COMPONENTS system)


## Generate services in the 'srv' folder
 add_service_files(
   FILES
   getGrasps.srv

 )

generate_messages(DEPENDENCIES geometry_msgs std_msgs grasp_execution)

catkin_package(
  INCLUDE_DIRS
      include
  LIBRARIES
  sq_grasping
  create_grasps
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(create_grasps src/sq_grasping/create_grasps.cpp)
target_link_libraries(create_grasps ${catkin_LIBRARIES} )


add_library(sq_grasping  src/sq_grasping/sq_grasping.cpp)
target_link_libraries(sq_grasping create_grasps ${catkin_LIBRARIES} ${PCL_INCLUDE_DIR})


install(TARGETS sq_grasping
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS create_grasps
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


add_executable(sq_grasping_node src/nodes/sq_grasping_node.cpp)
add_dependencies(sq_grasping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sq_grasping_node sq_grasping  ${catkin_LIBRARIES})

add_executable(execute_grasps_node src/nodes/execute_grasps_node.cpp)
add_dependencies(execute_grasps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(execute_grasps_node sq_grasping  ${catkin_LIBRARIES})


add_executable(get_grasp_client src/test/get_grasp_client.cpp)
add_dependencies(get_grasp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_grasp_client  ${catkin_LIBRARIES})

