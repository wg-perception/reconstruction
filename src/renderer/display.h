/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <boost/format.hpp>

#include <eigen3/Eigen/Eigen>

#include <GL/gl.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Model.h"

using Eigen::Matrix4d;


static double PI = 3.14159;


void
normalize_vector(float & x, float&y, float&z);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Display
{
public:
  Display();

  ~Display();

  static void
  load_model(const char * file_name);

  void
  set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near, double far);

  void reshape();

  void
  display();

  static void
  save_to_disk(GLuint fbo);

  static const Model &
  model()
  {
    return model_;
  }

  static const double
  near()
  {
    return near_;
  }

  static const double
  far()
  {
    return far_;
  }

  static const double
  focal_length_x()
  {
    return focal_length_x_;
  }

  static const double
  focal_length_y()
  {
    return focal_length_y_;
  }
//private:
  void clean_buffers();

  static unsigned int width_, height_;
  static double focal_length_x_, focal_length_y_, near_, far_;
  float angle_;

  static Matrix4d matrix_;
  static Model model_;
  GLuint scene_list_;
  /** The frame buffer object used for offline rendering */
  GLuint fbo_id_;
  /** The render buffer object used for offline depth rendering */
  GLuint rbo_id_;
  /** The render buffer object used for offline image rendering */
  GLuint texture_id_;
  aiVector3D scene_min_, scene_max_, scene_center_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Class that enables iterating viewpoint over a sphere.
 * This function is used to generate templates in LINE-MOD
 */
class DisplayIterator
{

};

#endif /* CAMERA_H_ */
