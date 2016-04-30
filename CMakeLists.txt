cmake_minimum_required(VERSION 2.8)
project(object_recognition_reconstruction)

find_package(catkin)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
# Remove vtkproj4 from PCL_LIBRARIES as bug
# https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=819741
list(REMOVE_ITEM PCL_LIBRARIES vtkproj4)

find_package(catkin REQUIRED cmake_modules ecto ecto_image_pipeline ecto_pcl
                             object_recognition_core opencv_candidate roscpp)
catkin_package(CATKIN_DEPENDS ecto ecto_image_pipeline ecto_pcl object_recognition_core opencv_candidate
               DEPENDS OpenCV PCL
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/mesh_object
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# add docs
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)
