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

#include "display.h"

unsigned int Display::image_width_ = 0, Display::image_height_ = 0;
double Display::focal_length_x_ = 0, Display::focal_length_y_ = 0, Display::near_ = 0, Display::far_ = 0;

Matrix4d Display::matrix_;
Model Display::model_;

void
Display::load_model(const char * file_name)
{
  model_.LoadModel(file_name);
}

void
Display::set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near,
                        double far)
{
  image_width_ = width;
  image_height_ = height;

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
}

void
Display::save_to_disk()
{
  cv::Mat image, depth, mask;

  image.create(cv::Size(image_width_, image_height_), CV_8UC3);
  depth.create(cv::Size(image_width_, image_height_), CV_32FC1);
  mask.create(cv::Size(image_width_, image_height_), CV_8UC1);
  glFlush();

  glReadPixels(0, 0, image_width_, image_height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());
  glReadPixels(0, 0, image_width_, image_height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

  float zNear = near_, zFar = far_;
  cv::Mat_<float>::iterator it = depth.begin<float>(), end = depth.end<float>();
  float max_allowed_z = zFar * 0.99;
  while (it != end)
  {
    //need to undo the depth buffer mapping
    //http://olivers.posterous.com/linear-depth-in-glsl-for-real
    *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
    ++it;
  }
  cv::Mat m(depth < max_allowed_z);
  m.copyTo(mask);
  depth.setTo(0, 255 - mask);

  static size_t index = 0;
  cv::Mat depth_scale(cv::Size(image_width_, image_height_), CV_16UC1);
  depth.convertTo(depth_scale, CV_16UC1, 1e3);

  cv::Mat depth_vis;
  depth.convertTo(depth_vis, CV_8UC1, 400);

  /*float min_x = 10000, max_x = 0;
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

  cv::imwrite(boost::str(boost::format("depth_%05d.png") % (index)), depth_scale);
  cv::imwrite(boost::str(boost::format("image_%05d.png") % (index)), image);
  cv::imwrite(boost::str(boost::format("mask_%05d.png") % (index)), mask);
  ++index;
}
