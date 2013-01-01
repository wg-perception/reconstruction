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

#ifndef RENDERER_GLUT_H_
#define RENDERER_GLUT_H_

// Make sure we define that so that we have FBO enabled
#define GL_GLEXT_PROTOTYPES

#include "renderer.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Class that displays a scene in a Frame Buffer Object
 * Inspired by http://www.songho.ca/opengl/gl_fbo.html
 */
class RendererGlut: public Renderer
{
public:
  /**
   * @param file_path the path of the mesh file
   */
  RendererGlut(const std::string & file_path)
      :
        Renderer(file_path)
  {
  }

  ~RendererGlut()
  {
    clean_buffers();
  }

private:
  virtual void
  clean_buffers();

  virtual void
  set_parameters_low_level();

  virtual void
  lookAt_low_level();

  virtual void
  render_low_level(cv::Mat &image, cv::Mat &depth) const;

  /** The frame buffer object used for offline rendering */
  GLuint fbo_id_;
  /** The render buffer object used for offline depth rendering */
  GLuint rbo_id_;
  /** The render buffer object used for offline image rendering */
  GLuint texture_id_;
};

#endif /* RENDERER_GLUT_H_ */
