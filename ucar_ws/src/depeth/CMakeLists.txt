# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(depeth)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp   image_transport nodelet sensor_msgs)
find_package(PCL REQUIRED) #配置PCL库
find_package( OpenCV REQUIRED)
set(CMAKE_C_STANDARD 99)
 
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(PkgConfig)






find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
cv_bridge
image_transport
)

catkin_package(
  CATKIN_DEPENDS
    roscpp


    image_transport
    sensor_msgs


  )

#message(STATUS "OpenCV library status:")
#message(STATUS "    version: ${OpenCV_VERSION}")
#message(STATUS "    libraries: ${OpenCV_LIBS}")
#message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(
  include 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 

  "/usr/include/eigen3/"
  #/usr/include/opencv2
 
  )
  
link_directories(${catkin_LINK_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(depeth src/ht_depeth.cpp)


target_link_libraries(depeth  ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES} ${catkin_LIBRARIES}  
                      ${OpenCV_LIBS} #/usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so
                      )



install(TARGETS depeth 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
#install(TARGETS lsd_node 
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
