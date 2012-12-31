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

#include "renderer.h"

#include <iostream>
#include <stdlib.h>

#include <GL/glut.h>

Display::Display(const std::string & file_name)
    :
      angle_(0),
      fbo_id_(0),
      rbo_id_(0),
      texture_id_(0),
      width_(0),
      height_(0),
      focal_length_x_(0),
      focal_length_y_(0),
      near_(0),
      far_(0),
      scene_list_(0)
{
  model_.LoadModel(file_name);

  // get a handle to the predefined STDOUT log stream and attach
  // it to the logging system. It remains active for all further
  // calls to aiImportFile(Ex) and aiApplyPostProcessing.
  ai_stream_ = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT, NULL);
  aiAttachLogStream(&ai_stream_);
}

Display::~Display()
{
  clean_buffers();
  // We added a log stream to the library, it's our job to disable it
  // again. This will definitely release the last resources allocated
  // by Assimp.
  aiDetachAllLogStreams();
}

void
Display::clean_buffers()
{
  if (texture_id_)
    glDeleteTextures(1, &texture_id_);
  texture_id_ = 0;

  // clean up FBO, RBO
  if (fbo_id_)
    glDeleteFramebuffers(1, &fbo_id_);
  fbo_id_ = 0;
  if (rbo_id_)
    glDeleteRenderbuffers(1, &rbo_id_);
  rbo_id_ = 0;
}

void
Display::set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near,
                        double far)
{
  width_ = width;
  height_ = height;

  focal_length_x_ = focal_length_x;
  focal_length_y_ = focal_length_y;

  near_ = near;
  far_ = far;

  /*  double f = focal_length_, w = image_width_, h = image_height_;

   Matrix4d K;
   K << f, 0, w / 2, 0, 0, f, h / 2, 0, 0, 0, 1, 0, 0, 0, 0, 1;

   matrix_ << 2 * K(0, 0) / w, -2 * K(0, 1) / w, (w - 2 * K(0, 2)) / w, 0, 0, -2 * K(1, 1) / h, (h - 2 * K(1, 2)) / h, 0, 0, 0, (-far
   - near)
   / (far - near), -2
   * far * near
   / (far - near), 0, 0, -1, 0;*/

  clean_buffers();

  int argc = 0;
  char **argv = 0;

  // Create an OpenGL context
  glutInit(&argc, argv);
  // By doing so, the window is not open
  glutInitDisplayMode(GLUT_DOUBLE);
  glutCreateWindow("Assimp - Very simple OpenGL sample");

  // create a framebuffer object
  glGenFramebuffers(1, &fbo_id_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

  // create a texture object
  glGenTextures(1, &texture_id_);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_id_, 0);

  // create a renderbuffer object to store depth info
  glGenRenderbuffers(1, &rbo_id_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_id_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo_id_);

  // Initialize the environment
  glClearColor(0.f, 0.f, 0.f, 1.f);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0); // Uses default lighting parameters

  glEnable(GL_DEPTH_TEST);

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable(GL_NORMALIZE);

  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

  // Initialize the projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  double fx = Display::focal_length_x();
  double fy = Display::focal_length_y();
  double fovy = 2 * atan(0.5 * height_ / fy) * 180 / CV_PI;
  double aspect = (width_ * fy) / (height_ * fx);

  // set perspective
  gluPerspective(fovy, aspect, near, far);
  glViewport(0, 0, width_, height_);
}

void
Display::display()
{
  float tmp;
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_id_);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  //gluLookAt(0.f, 0.f, 2.f, 0.f, 0.f, -5.f, 0.f, 1.f, 0.f);

  // rotate it around the y axis
  //glRotatef(angle, 0.f, 1.f, 0.f);

  // Figure a view point on the sphere
  static unsigned int n_points = 150;
  static unsigned int n = 0;
  // Angle in degrees
  static int angle_min = -80, angle_max = 80, angle_step = 40, angle = angle_min;
  static float radius_min = 0.4, radius_max = 0.8, radius_step = 0.2, radius = radius_min;

  static unsigned int index = 0;
  angle += angle_step;
  if (angle > angle_max)
  {
    angle = angle_min;
    radius += radius_step;
    if (radius > radius_max)
    {
      radius = radius_min;
      ++n;
      if (n >= n_points)
      {
        // Ugly hack to exit glutMainLoop
        throw("something");
      }
    }
  }
  unsigned int n_templates = ((angle_max - angle_min) / angle_step + 1) * n_points
                             * ((radius_max - radius_min) / radius_step + 1);

  // from http://www.xsi-blog.com/archives/115
  static float inc = CV_PI * (3 - sqrt(5));
  static float off = 2.0 / float(n_points);
  float y = n * off - 1 + (off / 2);
  float r = sqrt(1 - y * y);
  float phi = n * inc;
  float x = cos(phi) * r;
  float z = sin(phi) * r;

  float lat = acos(z);
  float lon;
  if ((abs(sin(lat)) < 1e-5) || (abs(y / sin(lat)) > 1))
    lon = 0;
  else
    lon = asin(y / sin(lat));

  x *= radius; // * cos(lon) * sin(lat);
  y *= radius; //float y = radius * sin(lon) * sin(lat);
  z *= radius; //float z = radius * cos(lat);

  // Figure out the up vector
  float x_up = radius * cos(lon) * sin(lat - 1e-5) - x;
  float y_up = radius * sin(lon) * sin(lat - 1e-5) - y;
  float z_up = radius * cos(lat - 1e-5) - z;
  normalize_vector(x_up, y_up, z_up);

  // Figure out the third vector of the basis
  float x_right = -y_up * z + z_up * y;
  float y_right = x_up * z - z_up * x;
  float z_right = -x_up * y + y_up * x;
  normalize_vector(x_right, y_right, z_right);

  // Rotate the up vector in that basis
  float angle_rad = angle * CV_PI / 180.;
  float x_new_up = x_up * cos(angle_rad) + x_right * sin(angle_rad);
  float y_new_up = y_up * cos(angle_rad) + y_right * sin(angle_rad);
  float z_new_up = z_up * cos(angle_rad) + z_right * sin(angle_rad);

  gluLookAt(x, y, z, 0.f, 0.f, 0.f, x_new_up, y_new_up, z_new_up);

  // scale the whole asset to fit into our view frustum
  aiVector3D scene_min, scene_max, scene_center;
  model_.get_bounding_box(&scene_min, &scene_max);
  scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
  scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
  scene_center.z = (scene_min.z + scene_max.z) / 2.0f;

  /*tmp = scene_max.x - scene_min.x;
   tmp = aisgl_max(scene_max.y - scene_min.y,tmp);
   tmp = aisgl_max(scene_max.z - scene_min.z,tmp);
   tmp = 1.f / tmp;
   glScalef(tmp, tmp, tmp);*/

  // center the model
  glTranslatef(-scene_center.x, -scene_center.y, -scene_center.z);

  // if the display list has not been made yet, create a new one and
  // fill it with scene contents
  if (scene_list_ == 0)
  {
    scene_list_ = glGenLists(1);
    glNewList(scene_list_, GL_COMPILE);
    // now begin at the root node of the imported data and traverse
    // the scenegraph by multiplying subsequent local transforms
    // together on GL's matrix stack.
    model_.Draw();
    glEndList();
  }

  glCallList(scene_list_);

  cv::Mat image_out, depth_out, mask_out;
  Display::render(image_out, depth_out, mask_out);

  ++index;
}

void
Display::render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out) const
{
  // Create images to copy the buffers to
  cv::Mat image;
  image.create(cv::Size(width_, height_), CV_8UC3);
  cv::Mat_<float> depth(height_, width_);

  cv::Mat_<uchar> mask = cv::Mat_<uchar>::zeros(cv::Size(width_, height_));

  glFlush();

  // Get data from the depth/image buffers
  glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo_id_);
  glReadBuffer(GL_DEPTH_ATTACHMENT);
  glReadPixels(0, 0, width_, height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, width_, height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

  float zNear = near_, zFar = far_;
  cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
  float max_allowed_z = zFar * 0.99;

  unsigned int i_min = width_, i_max = 0, j_min = height_, j_max = 0;
  for (unsigned int j = 0; j < height_; ++j)
    for (unsigned int i = 0; i < width_; ++i, ++it)
    {
      //need to undo the depth buffer mapping
      //http://olivers.posterous.com/linear-depth-in-glsl-for-real
      *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
      if (*it > max_allowed_z)
        *it = 0;
      else
      {
        mask(j, i) = 255;
        // Figure the inclusive bounding box of the mask
        if (j > j_max)
          j_max = j;
        else if (j < j_min)
          j_min = j;
        if (i > i_max)
          i_max = i;
        else if (i < i_min)
          i_min = i;
      }
    }

  // Rescale the depth to be in millimeters
  cv::Mat depth_scale(cv::Size(width_, height_), CV_16UC1);
  depth.convertTo(depth_scale, CV_16UC1, 1e3);

  // Crop the images, just so that they are smaller to write/read
  if (i_min > 0)
    --i_min;
  if (i_max < width_ - 1)
    ++i_max;
  if (j_min > 0)
    --j_min;
  if (j_max < height_ - 1)
    ++j_max;
  cv::Rect rect(i_min, j_min, i_max - i_min + 1, j_max - j_min + 1);

  depth_scale(rect).copyTo(depth_out);
  image(rect).copyTo(image_out);
  mask(rect).copyTo(mask_out);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RendererIterator::RendererIterator(const std::string & file_path)
    :
      renderer_(new Display(file_path))
{
}

void
RendererIterator::set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near,
                                 double far)
{
  renderer_->set_parameters(width, height, focal_length_x, focal_length_y, near, far);
}

void
RendererIterator::render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out)
{
  renderer_->display();
  renderer_->render(image_out, depth_out, mask_out);
}
