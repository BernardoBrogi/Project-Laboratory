cmake_minimum_required(VERSION 2.8.3)
project(lwr_simple_example)

find_package(catkin REQUIRED lwr_ros_client tf)


set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lwr_simple_example
  CATKIN_DEPENDS lwr_ros_client
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(include ${catkin_INCLUDE_DIRS}
"/home/hwadong/Youssef/sdk-3.7.3/include"
)

link_directories(
    "/home/hwadong/Youssef/sdk-3.7.3/lib/release/lin-x86_64-gcc"

)


add_library(${PROJECT_NAME} ${SRC_DIR}/simple_actions/linear_cart_action.cpp
                            ${INCLUDE_DIR}/simple_actions/linear_cart_action.h
                            ${SRC_DIR}/simple_actions/LfD_record.cpp
                            ${INCLUDE_DIR}/simple_actions/LfD_record.h
							${SRC_DIR}/simple_actions/LfD_replay.cpp
                            ${INCLUDE_DIR}/simple_actions/LfD_replay.h
                            ${SRC_DIR}/simple_actions/LfD_dmp.cpp
                            ${INCLUDE_DIR}/simple_actions/LfD_dmp.h
                            ${SRC_DIR}/simple_actions/Collaborate.cpp
                            ${INCLUDE_DIR}/simple_actions/Collaborate.h

        )
ign_import_target(UUID)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  
    -ldrd
    -ldhd
    -lusb-1.0
   )

add_executable(client_console_node  ${SRC_DIR}/client_console_node.cpp      )
target_link_libraries(client_console_node  ${catkin_LIBRARIES}     


         )


add_executable(client_action_node  ${SRC_DIR}/client_action_node.cpp	)
target_link_libraries(client_action_node  ${PROJECT_NAME} ${catkin_LIBRARIES})






