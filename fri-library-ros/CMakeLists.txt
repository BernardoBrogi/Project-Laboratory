cmake_minimum_required(VERSION 2.4.6)
project(fri_library_ros)

## NEW
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 
message(STATUS "CMAKE_CURRENT_BINARY_DIR:  ${CMAKE_CURRENT_BINARY_DIR}")
message("PROJECT_NAME:  ${PROJECT_NAME}")
message("PROJECT_SOURCE_DIR:  ${PROJECT_SOURCE_DIR}")

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fri_library_ros
  DEPENDS Eigen
#  DEPENDS Ode Bullet
)



set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
#set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(H_FILES ${INCLUDE_DIR}/FastResearchInterface.h
            ${INCLUDE_DIR}/fricomm.h
            ${INCLUDE_DIR}/friremote.h
            ${INCLUDE_DIR}/friudp.h)


## NEW
include_directories(include ${Eigen_INCLUDE_DIRS})
FILE(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${H_FILES} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} rt ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

# set_target_properties(${PROJECT_NAME}  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set_target_properties(${PROJECT_NAME}  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)






