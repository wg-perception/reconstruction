include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(ecto_pcl REQUIRED)
find_package(Eigen REQUIRED)
find_package(ecto_image_pipeline REQUIRED)
find_package(object_recognition_core REQUIRED)
find_package(PCL REQUIRED)

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


set(USE_ON_ROBOT False)
if (USE_ON_ROBOT)
  include_directories(SYSTEM /usr/include/suitesparse)

  #TODO: use strict diagnostics
  remove_definitions(-Wno-strict-aliasing -Wall -Werror)
  add_definitions(-fpermissive)

  #TODO: fix
  include_directories(../../opencv_candidate/src/rgbd/include/)

  include_directories(/opt/ros/groovy/include ${g2o_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${CHOLMOD_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

  #publish odometry messages
  include_directories(/opt/ros/fuerte/stacks/geometry/tf/include/
                      /opt/ros/fuerte/stacks/bullet/include/
                      /opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include/
  )

  include_directories(/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include/
                      /opt/ros/fuerte/stacks/image_common/image_transport/include/
  )

  find_package(ROS COMPONENTS roscpp)
  add_executable(robot_demo robot_demo/robot_demo.cpp
                            robot_demo/poseRT.cpp

#TODO: link with a library after splitting the code
                            ../../opencv_candidate/src/rgbd/samples/model_capture/model_capture.hpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/ocv_pcl_eigen_convert.hpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/create_optimizer.hpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/load_data.cpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/mask_data.cpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/frame_to_frame.cpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/graph_se3.cpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/graph_se3_rgbd_icp.cpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/graph_se3_landmarks.cpp
                            ../../opencv_candidate/src/rgbd/samples/model_capture/show_model.cpp
  )

  link_directories(/opt/ros/groovy/lib)
  target_link_libraries(robot_demo ${roscpp_LIBRARIES}
                                   ${OpenCV_LIBRARIES}
                                   /usr/lib/libboost_signals.so
                                   /opt/ros/fuerte/lib/libmessage_filters.so
                                   /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/lib/libcv_bridge.so
                                   /opt/ros/fuerte/stacks/image_common/image_transport/lib/libimage_transport.so

                                   #publish odometry messages
                                   /opt/ros/fuerte/stacks/geometry/tf/lib/libtf.so

                                   opencv_rgbd
#TODO: link with the library after splitting the code
                                   /usr/lib/libboost_thread.so
                                   ${PCL_LIBRARIES}
                                   /opt/ros/groovy/lib/libg2o_core.so
                                   /opt/ros/groovy/lib/libg2o_types_slam3d.so
                                   /opt/ros/groovy/lib/libg2o_types_icp.so
                                   /opt/ros/groovy/lib/libg2o_stuff.so
                                   cholmod
                                   cxsparse
                                   ${CHOLMOD_LIBRARIES}
                                   ${CSPARSE_LIBRARY} /opt/ros/groovy/lib/libg2o_csparse_extension.so
  )
endif()
