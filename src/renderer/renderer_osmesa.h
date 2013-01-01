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

#ifndef RENDERER_OSMESA_H_
#define RENDERER_OSMESA_H_

#include "renderer.h"

#include <GL/osmesa.h>
#include <GL/gl.h>
#include <GL/glu.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Class that displays a scene ina Frame Buffer Object
 * Inspired by http://www.songho.ca/opengl/gl_fbo.html
 */
class RendererOSMesa: public Renderer
{
public:
  /**
   * @param file_path the path of the mesh file
   */
  RendererOSMesa(const std::string & mesh_path);

  ~RendererOSMesa()
  {
    clean_buffers();
  }

private:
  virtual void
  clean_buffers();

  virtual void
  set_parameters_low_level();

  virtual void
  bind_buffers() const;

  /** Off-Screen mesa context for pure software rendering */
  OSMesaContext ctx_;
  /** The RGBA buffer when doing software rendering */
  void *ctx_buffer_;
};

#endif /* CAMERA_H_ */
