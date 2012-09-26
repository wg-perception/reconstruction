cmake_minimum_required(VERSION 2.8)
project(object_recognition_reconstruction)

find_package(catkin)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package()
else()
catkin_stack()
catkin_project(object_recognition_reconstruction)
endif()

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
endif()
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

