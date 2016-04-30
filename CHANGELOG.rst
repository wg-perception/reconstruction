0.3.6 (2016-04-30)
------------------
* fix PCL compilation
* Fix poisson script. Enhance ball pivoting script
* Contributors: JimmyDaSilva, Vincent Rabaud

0.3.5 (2016-01-01)
------------------
* Fix the bad conversion dependency.
  ecto_image_pipelne is not compiled with PCL anymore, hence the
  loss of the conversions cells. Fixes `#6 <https://github.com/wg-perception/reconstruction/issues/6>`_
* Contributors: Vincent Rabaud

0.3.4 (2015-04-20)
------------------
* re-writing of #5 without sudo. Needs testing
* Add a missing parameter to the Laplacian Smooth filter (cotangentWeight)
* clean extensions
* Contributors: Jorge Santos Simón, Vincent Rabaud

0.3.3 (2014-12-14)
------------------
* Merge pull request `#3 <https://github.com/wg-perception/reconstruction/issues/3>`_ from garaemon/fix-obs-ids-tuple
  cast obs_ids to list
* cast obs_ids to list
* Contributors: Ryohei Ueda, Vincent Rabaud

0.3.2 (2014-11-21)
------------------
* use local code to convert OpenCV to Eigen
* Contributors: Vincent Rabaud

0.3.1 (2014-07-27)
------------------
* no need to depend on OpenCV as we get it by transition
* get the code to compile under Indigo
* Contributors: Vincent Rabaud

* no need to depend on OpenCV as we get it by transition
* get the code to compile under Indigo
* Contributors: Vincent Rabaud

0.3.0 (2013-12-07 13:13:06 -0800)
---------------------------------
- drop Fuerte support
- fix meshlab dependency
