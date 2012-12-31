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

#include "model.h"

using Eigen::Matrix4d;

/** Function that normalizes a vector
 * @param x the x component of the vector
 * @param y the y component of the vector
 * @param z the z component of the vector
 */
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
class Renderer
{
public:
  /**
   * @param file_path the path of the mesh file
   */
  Renderer(const std::string & file_path);

  ~Renderer();

  void
  set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near, double far);

  /** Similar to the gluLookAt function
   * @param x the x position of the eye pointt
   * @param y the y position of the eye point
   * @param z the z position of the eye point
   * @param upx the x direction of the up vector
   * @param upy the y direction of the up vector
   * @param upz the z direction of the up vector
   */
  void
  lookAt(GLdouble x, GLdouble y, GLdouble z, GLdouble upx, GLdouble upy, GLdouble upz);

  /** Renders the content of the current OpenGL buffers to images
   * @param image_out the RGB image
   * @param depth_out the depth image
   * @param mask_out the mask image
   */
  void
  render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out) const;

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
  RendererIterator(const std::string & file_path, size_t n_points);

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
  isDone() const
  {
    return (index_ >= n_points_);
  }

  void
  render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out);
private:
  /** The number of points on the sphere */
  size_t n_points_;
  /** The index of the view point we are at now */
  size_t index_;
  /** The renderer object containing the scene and that will render images */
  cv::Ptr<Renderer> renderer_;
  /** Values for the angle sampling in degrees */
  int angle_min_, angle_max_, angle_step_, angle_;
  /** Values for the scale sampling */
  float radius_min_, radius_max_, radius_step_, radius_;
};

#endif /* CAMERA_H_ */
