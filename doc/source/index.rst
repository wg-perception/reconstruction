.. _reconstruction:

object_recognition_reconstruction: 3D Object Reconstruction
###########################################################

``Reconstruction`` provides a utility to create a 3d reconstruction of an object. For now, it creates an approximate untextured mesh from captured data, that is good enough for grasping. The algorithm simply merges the depth imags from several views and smoothes them a bit. Nothing smarted is done.


Install dependencies
********************

First install meshlab as it is used to go from the aggregation of depth maps to a mesh:

   .. code-block:: sh

      sudo apt-get install meshlab

Command Line
************

From the command line, you want to execute the following command to compute all meshes and commit them to the local DB:

.. toggle_table::
    :arg1: Non-ROS
    :arg2: ROS

.. toggle:: Non-ROS

   .. code-block:: sh

      ./apps/mesh_object --all --visualize --commit

.. toggle:: ROS
   
   .. code-block:: sh

      rosrun object_recognition_reconstruction mesh_object --all --visualize --commit

Or tune your parameters using the appropriate command line arguments:

.. program-output:: ../../apps/mesh_object --help
    :prompt:
    :in_srcdir:

Web Interface
*************

Reconstruction also provides a web interface to visualize the different meshes. In your build folder, just run:

.. code-block:: sh

    make or_web_ui

You can then visualize the meshes here: `http://localhost:5984/or_web_ui/_design/viewer/index.html <http://localhost:5984/or_web_ui/_design/viewer/index.html>`_

Tips
****

To create a textured mesh from a colored point cloud in meshlab: http://www.youtube.com/watch?v=JzmODsVQV7w
