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
        angle_(0)
  {
  }

  static void
  load_model(const char * file_name);

  static void
  set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double near, double far);

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

  static void
  save_to_disk();

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

  static unsigned int image_width_, image_height_;
  static double focal_length_x_, focal_length_y_, near_, far_;
  float angle_;

  static Matrix4d matrix_;
  static Model model_;
  GLuint scene_list_;
  aiVector3D scene_min_, scene_max_, scene_center_;
};

#endif /* CAMERA_H_ */
