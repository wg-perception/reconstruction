find_package(Eigen REQUIRED)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${EIGEN_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)

# deal with the reconstruction module
ectomodule(object_recognition_reconstruction DESTINATION ./
                                             INSTALL
  module.cpp
  #surfel_conversion.cpp
  #surfels.cpp
  reconstruction_db.cpp
  #SurfelReconstruction.cpp
  PointCloudTransform.cpp
  PointCloudAccumulator.cpp
  PointCloudMesh.cpp
)

link_ecto(object_recognition_reconstruction ${BOOST_LIBRARIES}
                                            ${catkin_LIBRARIES}
                                            ${OpenCV_LIBRARIES}
                                            ${PCL_LIBRARIES}
)


set(USE_ON_ROBOT True)
if (USE_ON_ROBOT)
  include_directories(SYSTEM /usr/include/suitesparse)

  #TODO: use strict diagnostics
  remove_definitions(-Wno-strict-aliasing -Wall -Werror)
  add_definitions(-fpermissive)

  include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

  add_executable(robot_demo robot_demo/robot_demo.cpp
                            robot_demo/poseRT.cpp
                            robot_demo/rosUtils.cpp
                            robot_demo/modelCaptureInterface.cpp
                            robot_demo/dataGrabbing.cpp

#TODO: link with a library after splitting the code


                            ../../opencv_candidate/src/rgbd/samples/reconst3d/capture.cpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/graph_optimizations.hpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/graph_se3.cpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/graph_se3_rgbd_icp.cpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/graph_se3_rgbd_icp_model.cpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/load_data.cpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/mask_data.cpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/model.cpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/ocv_pcl_convert.hpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/reconst3d.hpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/reconst3d_init.cpp
                            ../../opencv_candidate/src/rgbd/samples/reconst3d/reconstruction.cpp
  )

  find_package(Boost REQUIRED signals)
  target_link_libraries(robot_demo ${Boost_LIBRARIES}
                                   ${catkin_LIBRARIES}
                                   ${OpenCV_LIBRARIES}
                                   ${PCL_LIBRARIES}
  )
  set_target_properties(robot_demo PROPERTIES COMPILE_FLAGS "-fopenmp" LINK_FLAGS "-fopenmp")
endif()
