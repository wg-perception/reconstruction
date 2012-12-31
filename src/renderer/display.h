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

#include <string>

#include <eigen3/Eigen/Eigen>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

#include <opencv2/core/core.hpp>

#include "Model.h"

using Eigen::Matrix4d;

template<typename T>
void
normalize_vector(T & x, T&y, T&z)
{
  T norm = std::sqrt(x * x + y * y + z * z);
  x /= norm;
  y /= norm;
  z /= norm;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Class that displays a scene ina Frame Buffer Object
 * Inspired by http://www.songho.ca/opengl/gl_fbo.html
 */
class Display
{
public:
  /**
   * @param file_path the path of the mesh file
   */
  Display(const std::string & file_path);

  ~Display();

  void
  set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near, double far);

  void
  display();

  void
  render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out) const;

  const Model &
  model() const
  {
    return model_;
  }

  const double
  near()
  {
    return near_;
  }

  const double
  far()
  {
    return far_;
  }

  const double
  focal_length_x()
  {
    return focal_length_x_;
  }

  const double
  focal_length_y()
  {
    return focal_length_y_;
  }

private:
  void
  clean_buffers();

  unsigned int width_, height_;
  double focal_length_x_, focal_length_y_, near_, far_;
  float angle_;

  Matrix4d matrix_;
  Model model_;
  GLuint scene_list_;
  /** The frame buffer object used for offline rendering */
  GLuint fbo_id_;
  /** The render buffer object used for offline depth rendering */
  GLuint rbo_id_;
  /** The render buffer object used for offline image rendering */
  GLuint texture_id_;
  aiVector3D scene_min_, scene_max_, scene_center_;
  aiLogStream ai_stream_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Class that enables iterating the viewpoint over a sphere.
 * This function is used to generate templates in LINE-MOD
 */
class RendererIterator
{
public:
  /**
   * @param file_path the path of the mesh to render
   */
  RendererIterator(const std::string & file_path);

  void
  set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near, double far);

  /** Iterate to get to a different view
   * We don't implement the postfix operator on purpose
   * @return an incremented version of itself
   */
  RendererIterator &
  operator++();

  /**
   * @return true if we are done with all the views, false otherwise
   */
  bool
  isDone() const;

  void
  render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out);
private:
  cv::Ptr<Display> renderer_;
};

#endif /* CAMERA_H_ */
