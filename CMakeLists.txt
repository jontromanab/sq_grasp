cmake_minimum_required(VERSION 2.8.3)
project(sq_grasp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

find_package(PCL 1.8 REQUIRED)

#find_package(octomap REQUIRED)

 #add_message_files(
   #FILES
   #capShape.msg
   #capability.msg
   #WsSphere.msg
   #WorkSpace.msg

 #)


 #generate_messages(
   #DEPENDENCIES
   #geometry_msgs std_msgs
 #)


catkin_package(

  INCLUDE_DIRS
    include
  LIBRARIES
    segmentation
  CATKIN_DEPENDS #message_runtime
)


IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIR})



 add_library(segmentation
   src/sq_grasp/segmentation.cpp
   )




 target_link_libraries(segmentation
   ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARY_DIRS}
 )


 install(TARGETS segmentation
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

 )


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )



#have to create a test file
add_executable(segmentation_test src/tests/segmentation_test.cpp)
add_dependencies(segmentation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(segmentation_test segmentation  ${catkin_LIBRARIES})


#executable for node
add_executable(segmentation_node src/nodes/segmentation_node.cpp)
add_dependencies(segmentation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(segmentation_node segmentation  ${catkin_LIBRARIES})

