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

#include <eigen3/Eigen/Eigen>

#include <GL/gl.h>
#include <GL/glut.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Model.h"

using Eigen::Matrix4d;

class Display
{
public:
  Display()
      :
        image_width_(900),
        image_height_(600),
        angle_(0)
  {
    near_ = 0.1;
    far_ = 5.0;

    double f = focal_length_, w = image_width_, h = image_height_, far = far_, near = near_;

    Matrix4d K;
    K << f, 0, w / 2, 0, 0, f, h / 2, 0, 0, 0, 1, 0, 0, 0, 0, 1;

    matrix_ << 2 * K(0, 0) / w, -2 * K(0, 1) / w, (w - 2 * K(0, 2)) / w, 0, 0, -2 * K(1, 1) / h, (h - 2 * K(1, 2)) / h, 0, 0, 0, (-far
        - near)
                                                                                                                                 / (far - near), -2
        * far * near
                                                                                                                                                 / (far - near), 0, 0, -1, 0;
  }

  void
  load_model(const char * file_name)
  {
    model_.LoadModel(file_name);
  }

  void
  display(void)
  {
    float tmp;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.f, 0.f, 3.f, 0.f, 0.f, -5.f, 0.f, 1.f, 0.f);

    // rotate it around the y axis
    glRotatef(angle_, 0.f, 1.f, 0.f);

    // scale the whole asset to fit into our view frustum
    model_.get_bounding_box(&scene_min_, &scene_max_);
    scene_center_.x = (scene_min_.x + scene_max_.x) / 2.0f;
    scene_center_.y = (scene_min_.y + scene_max_.y) / 2.0f;
    scene_center_.z = (scene_min_.z + scene_max_.z) / 2.0f;

    tmp = scene_max_.x - scene_min_.x;
    tmp = aisgl_max(scene_max_.y - scene_min_.y, tmp);
    tmp = aisgl_max(scene_max_.z - scene_min_.z, tmp);
    tmp = 1.f / tmp;
    glScalef(tmp, tmp, tmp);

    // center the model
    glTranslatef(-scene_center_.x, -scene_center_.y, -scene_center_.z);

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

    glutSwapBuffers();

    // Save the interesting data to disk
    save_to_disk();

    do_motion();
  }

  void
  save_to_disk()
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
    while (it != end)
    {
      //need to undo the depth buffer mapping
      //http://olivers.posterous.com/linear-depth-in-glsl-for-real
      *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
      ++it;
    }
    cv::Mat m(depth < (zFar * 0.99));
    m.copyTo(mask);

    cv::imwrite("depth.png", depth);
    cv::imwrite("image.png", image);
    cv::imwrite("mask.png", mask);
  }

  const Model &
  model() const
  {
    return model_;
  }
private:
  void
  do_motion(void)
  {
    static GLint prev_time = 0;

    int time = glutGet(GLUT_ELAPSED_TIME);
    angle_ += (time - prev_time) * 0.01;
    prev_time = time;

    glutPostRedisplay();
  }

  unsigned image_width_, image_height_;
  double focal_length_, near_, far_;
  float angle_;

  Matrix4d matrix_;
  Model model_;
  GLuint scene_list_;
  aiVector3D scene_min_, scene_max_, scene_center_;
};

#endif /* CAMERA_H_ */
