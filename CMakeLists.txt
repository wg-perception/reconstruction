cmake_minimum_required(VERSION 2.8)
project(object_recognition_reconstruction)

find_package(catkin REQUIRED ecto ecto_image_pipeline ecto_pcl object_recognition_core)
find_package(PCL REQUIRED)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package(DEPENDS ecto ecto_image_pipeline ecto_pcl object_recognition_core)
else()
find_package(ecto)
find_package(ecto_pcl)
catkin_stack()
catkin_project(object_recognition_reconstruction DEPENDS ecto ecto_image_pipeline ecto_pcl object_recognition_core)
endif()

find_package(ecto_pcl)
find_package(ecto_image_pipeline)
find_package(object_recognition_core)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${object_recognition_core_INCLUDE_DIRS} ${ecto_pcl_INCLUDE_DIRS} ${ecto_image_pipeline_INCLUDE_DIRS})

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
