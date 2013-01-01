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

#include "renderer_glut.h"

#include <iostream>
#include <stdlib.h>

#include <GL/glut.h>

void
RendererGlut::clean_buffers()
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
RendererGlut::set_parameters_low_level()
{
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
}

void
RendererGlut::bind_buffers() const
{
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_id_);
}
