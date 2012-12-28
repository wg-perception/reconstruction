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

#define GL_GLEXT_PROTOTYPES
#include "display.h"

#include <iostream>
#include <stdlib.h>

#include <GL/glut.h>

void
normalize_vector(float & x, float&y, float&z)
{
  float norm = std::sqrt(x * x + y * y + z * z);
  x /= norm;
  y /= norm;
  z /= norm;
}

Display::Display()
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
}

Display::~Display()
{
  clean_buffers();
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
Display::load_model(const char * file_name)
{
  model_.LoadModel(file_name);
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

  // create a texture object
  glGenTextures(1, &texture_id_);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE); // automatic mipmap generation included in OpenGL v1.4
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
  glBindTexture(GL_TEXTURE_2D, 0);

  // create a framebuffer object, you need to delete them when program exits.
  glGenFramebuffers(1, &fbo_id_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

  // create a renderbuffer object to store depth info
  // NOTE: A depth renderable image should be attached the FBO for depth test.
  // If we don't attach a depth renderable image to the FBO, then
  // the rendering output will be corrupted because of missing depth test.
  // If you also need stencil test for your rendering, then you must
  // attach additional image to the stencil attachement point, too.
  glGenRenderbuffers(1, &rbo_id_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_id_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  // attach a texture to FBO color attachement point
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_id_, 0);

  // attach a renderbuffer to depth attachment point
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo_id_);
}

void
Display::reshape()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  double fx = Display::focal_length_x();
  double fy = Display::focal_length_y();
  double fovy = 2 * atan(0.5 * height_ / fy) * 180 / PI;
  double aspect = (width_ * fy) / (height_ * fx);

  // define the near and far clipping planes
  double near = Display::near();
  double far = Display::far();

  // set perspective
  gluPerspective(fovy, aspect, near, far);
  glViewport(0, 0, width_, height_);
}

void
Display::display()
{
  float tmp;
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

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
  std::cout << n_templates << " templates: " << float(index * 100) / float(n_templates) << std::endl;
  //std::cout << angle << " " << lon << " " << lat << " " << radius << std::endl;

  // from http://www.xsi-blog.com/archives/115
  static float inc = PI * (3 - sqrt(5));
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
  float angle_rad = angle * PI / 180.;
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

  if (index > 0)
    Display::save_to_disk(fbo_id_);

  ++index;
}

void
Display::save_to_disk(GLuint fbo) const
{
  cv::Mat image, depth;
  image.create(cv::Size(width_, height_), CV_8UC3);
  depth.create(cv::Size(width_, height_), CV_32FC1);

  cv::Mat_<uchar> mask = cv::Mat_<uchar>::zeros(cv::Size(width_, height_));

  glFlush();

  glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
  glReadBuffer(GL_DEPTH_ATTACHMENT);
  glReadPixels(0, 0, width_, height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());
  glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);

  glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, width_, height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());
  glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);

  float zNear = near_, zFar = far_;
  cv::Mat_<float>::iterator it = depth.begin<float>(), end = depth.end<float>();
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

  static size_t index = 0;
  cv::Mat depth_scale(cv::Size(width_, height_), CV_16UC1);
  depth.convertTo(depth_scale, CV_16UC1, 1e3);

  /*cv::Mat depth_vis;
   depth.convertTo(depth_vis, CV_8UC1, 400);

   float min_x = 10000, max_x = 0;
   float min_y = 10000, max_y = 0;
   float min_z = 10000, max_z = 0;

   // From http://nehe.gamedev.net/article/using_gluunproject/16013/
   GLint viewport[4];
   GLdouble modelview[16];
   GLdouble projection[16];
   GLfloat winX, winY, winZ;
   GLdouble posX, posY, posZ;

   glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
   glGetDoublev(GL_PROJECTION_MATRIX, projection);
   glGetIntegerv(GL_VIEWPORT, viewport);

   for(unsigned int j=0;j<4;++j)
   std::cout << viewport[j] << " ";
   std::cout << std::endl;
   for(unsigned int i=0;i<16;++i) {
   modelview[i] = 0;
   std::cout << modelview[i] << " ";
   }
   modelview[0] = 1;
   modelview[5] = 1;
   modelview[10] = 1;
   modelview[15] = 1;
   std::cout << std::endl;
   for(unsigned int j=0;j<16;++j)
   std::cout << projection[j] << " ";
   std::cout << std::endl;

   it = depth.begin<float>();
   for (unsigned int y = 0; y < image_height_; ++y)
   for (unsigned int x = 0; x < image_width_; ++x, ++it)
   {
   if ((*it >= max_allowed_z) || (*it == 0))
   continue;
   winX = (float) x;
   winY = (float) viewport[3] - (float) y;
   glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

   gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
   std::cout << posZ << " " << (*it) << std::endl;
   min_x = std::min(min_x, float(posX));
   max_x = std::max(max_x, float(posX));
   min_y = std::min(min_y, float(posY));
   max_y = std::max(max_y, float(posY));
   min_z = std::min(min_z, float(posZ));
   max_z = std::max(max_z, float(posZ));
   }
   std::cout << min_x << " - " << max_x << " ---- " << min_y << " - " << max_y << " ---- " << min_z << " - " << max_z
   << std::endl;

   cv::namedWindow("toto");
   cv::namedWindow("toto2");
   cv::imshow("toto", depth_vis);
   cv::imshow("toto2", mask);
   cv::waitKey(0);*/

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

  cv::imwrite(boost::str(boost::format("depth_%05d.png") % (index)), depth_scale(rect));
  cv::imwrite(boost::str(boost::format("image_%05d.png") % (index)), image(rect));
  cv::imwrite(boost::str(boost::format("mask_%05d.png") % (index)), mask(rect));
  ++index;
}
