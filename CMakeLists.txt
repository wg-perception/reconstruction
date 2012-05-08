cmake_minimum_required(VERSION 2.8)
project(object_recognition_reconstruction)

find_package(catkin)
catkin_stack()
catkin_project(object_recognition_reconstruction)

find_package(ecto REQUIRED)
find_pcl_package()
set_ecto_install_package_name()

find_package(object_recognition_core REQUIRED)

#set_ecto_install_package_name()

add_subdirectory(src)

install(DIRECTORY apps/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        PATTERN "*.bag" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
)

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()
