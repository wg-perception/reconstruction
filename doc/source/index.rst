Reconstruction
==============

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
