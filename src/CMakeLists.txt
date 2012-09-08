include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(ecto_pcl REQUIRED)
find_package(Eigen REQUIRED)
find_package(ecto_image_pipeline REQUIRED)
find_package(object_recognition_core REQUIRED)

find_pcl_package()

include_directories(SYSTEM
                    ${ecto_pcl_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
                    ${ecto_image_pipeline_INCLUDE_DIRS}
                    ${object_recognition_core_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

#deal with the objcog module
ectomodule(object_recognition_reconstruction INSTALL
  module.cpp
  #surfel_conversion.cpp
  #surfels.cpp
  reconstruction_db.cpp
  #SurfelReconstruction.cpp
  PointCloudTransform.cpp
  PointCloudAccumulator.cpp
  PointCloudMesh.cpp
  )

set_target_properties(object_recognition_reconstruction_ectomodule
      PROPERTIES
      COMPILE_FLAGS "-Wno-unknown-pragmas -Wno-unused-but-set-variable"
      )

if (ROS_GROOVY_FOUND)
  find_package(catkin REQUIRED roscpp)
endif()
link_ecto(object_recognition_reconstruction
          ${object_recognition_core_LIBRARIES}
          ${PCL_LIBRARIES}
          ${OpenCV_LIBS}
          ${OpenCV_LIBRARIES}
          ${roscpp_LIBRARIES}
)
