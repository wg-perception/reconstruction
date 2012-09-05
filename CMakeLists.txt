cmake_minimum_required(VERSION 2.8)
project(object_recognition_reconstruction)

find_package(catkin)
catkin_stack()
catkin_project(object_recognition_reconstruction)

find_package(ecto REQUIRED)
find_pcl_package()

find_package(object_recognition_core REQUIRED)

add_subdirectory(src)

if (ROS_ELECTRIC_FOUND)
install(DIRECTORY apps/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        PATTERN "*.bag" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        PATTERN "roscompat*" EXCLUDE
)
else()
# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/mesh_object
        DESTINATION share/${PROJECT_NAME}/bin
)
endif()

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

