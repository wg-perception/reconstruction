cmake_minimum_required(VERSION 2.8)
project(object_recognition_reconstruction)

find_package(catkin)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED ecto ecto_image_pipeline ecto_pcl
                             object_recognition_core opencv_candidate roscpp)
catkin_package(CATKIN_DEPENDS ecto ecto_image_pipeline ecto_pcl object_recognition_core opencv_candidate
               DEPENDS OpenCV PCL
)
else()
find_package(ecto)
find_package(ecto_pcl)
catkin_stack()
find_package(ecto_pcl)
find_package(ecto_image_pipeline)
find_package(object_recognition_core)
find_package(opencv_candidate)
find_package(roscpp)
catkin_project(object_recognition_reconstruction DEPENDS ecto ecto_image_pipeline ecto_pcl object_recognition_core)
list(APPEND catkin_LIBRARIES #/opt/ros/fuerte/lib/libmessage_filters.so
#                             /opt/ros/fuerte/stacks/image_common/image_transport/lib/libimage_transport.so
#                             /opt/ros/fuerte/stacks/geometry/tf/lib/libtf.so
#                             /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/lib/libcv_bridge.so
                             ${opencv_candidate_LIBRARIES}
                             ${roscpp_LIBRARIES}
)
list(APPEND catkin_INCLUDE_DIRS #/opt/ros/fuerte/stacks/geometry/tf/include/
                                #/opt/ros/fuerte/stacks/bullet/include/
                                #/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include/
                                #/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include/
                                #/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include/
                                #/opt/ros/fuerte/stacks/image_common/image_transport/include/
                                ${ecto_image_pipeline_INCLUDE_DIRS}
                                ${object_recognition_core_INCLUDE_DIRS}
)
endif()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

if (ROS_FUERTE_FOUND)
# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/mesh_object
        DESTINATION share/${PROJECT_NAME}/bin
)
endif()
if (ROS_GROOVY_OR_ABOVE_FOUND)
# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/mesh_object
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()
