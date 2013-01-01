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

#include "renderer_osmesa.h"

#include <iostream>
#include <stdlib.h>

#include <GL/glut.h>

/**
 * @param file_path the path of the mesh file
 */
RendererOSMesa::RendererOSMesa(const std::string & mesh_path)
    :
      Renderer(mesh_path),
      ctx_(0),
      ctx_buffer_(0)
{
}

void
RendererOSMesa::clean_buffers()
{
  if (ctx_)
    OSMesaDestroyContext(ctx_);

  if (ctx_buffer_)
  {
    free(ctx_buffer_);
    ctx_buffer_ = 0;
  }
}

void
RendererOSMesa::set_parameters_low_level()
{
  ctx_ = OSMesaCreateContextExt(OSMESA_RGB, 32, 0, 0, NULL);

  ctx_buffer_ = malloc(width_ * height_ * 3 * sizeof(GLubyte));
  OSMesaMakeCurrent(ctx_, ctx_buffer_, GL_UNSIGNED_BYTE, width_, height_);
}

void
RendererOSMesa::bind_buffers() const
{
}
