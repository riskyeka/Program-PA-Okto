cmake_minimum_required(VERSION 2.8.3)
project(ersow_comm)
add_compile_options(-std=c++11)

find_package (catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  message_generation
  vision
)

add_message_files (DIRECTORY msg
	FILES
	basestation.msg
	baseMsg.msg
	dataAgentMsg.msg
	pub_ai.msg
)

generate_messages (
	DEPENDENCIES 
	std_msgs
)

catkin_package (
#  INCLUDE_DIRS src/ratdma
#  INCLUDE_DIRS src/rtdb  
#  LIBRARIES ersow_comm
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	
add_executable(broker_node
	src/broker.cpp)
target_link_libraries(broker_node ${catkin_LIBRARIES})

add_executable(ratdma_node 
	src/ratdma.cpp
	src/ratdma/multicast.h)
target_link_libraries(ratdma_node ${catkin_LIBRARIES})



# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

