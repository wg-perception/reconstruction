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
find_package(ROS REQUIRED ecto ecto_image_pipeline ecto_pcl object_recognition_core opencv_candidate roscpp)

catkin_stack()
catkin_project(object_recognition_reconstruction DEPENDS ecto ecto_image_pipeline ecto_pcl object_recognition_core)

set(catkin_LIBRARIES ${ROS_LIBRARIES})
set(catkin_INCLUDE_DIRS ${ROS_INCLUDE_DIRS})
set(CATKIN_PACKAGE_BIN_DESTINATION share/${PROJECT_NAME}/bin)
endif()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/mesh_object
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

# add docs
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)
