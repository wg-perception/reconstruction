object_recognition_reconstruction: 3D Object Reconstruction
===========================================================

``Reconstruction`` provides a utility to create a 3d reconstruction of an object. For now, it creates
an approximate untextured mesh from captured data, that is good enough for grasping.

Command Line
------------

From the command line, you want to execute the following command to compute all meshes and commit them to the local DB:

.. code-block:: sh

    mesh_object --all --visualize --commit

Or tune your parameters using the appropriate command line arguments:
    
.. program-output:: ../../apps/mesh_object --help
    :prompt:
    :in_srcdir:

Web Interface
-------------

Reconstruction also provides a web interface to visualize the different meshes. In your build folder, just run:

.. code-block:: sh

    make or_web_ui

You can then visualize the meshes here: `http://localhost:5984/or_web_ui/_design/viewer/index.html <http://localhost:5984/or_web_ui/_design/viewer/index.html>`_

Tips
----

To create a textured mesh from a colored point cloud in meshlab:
http://www.youtube.com/watch?v=JzmODsVQV7w
