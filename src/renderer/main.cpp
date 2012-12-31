//
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c), assimp OpenGL sample
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

// ----------------------------------------------------------------------------
// Simple sample to prove that Assimp is easy to use with OpenGL.
// It takes a file name as command line parameter, loads it using standard
// settings and displays it.
// ----------------------------------------------------------------------------

#define GL_GLEXT_PROTOTYPES

#include <iostream>
#include <stdlib.h>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "renderer.h"

int
main(int argc, char **argv)
{
  // Define the display
  size_t width = 640, height = 480;
  double near = 0.1, far = 1000;
  double focal_length_x = 525, focal_length_y = 525;

  // the model name can be specified on the command line.
  Display display = Display(std::string(argv[1]));

  display.set_parameters(width, height, focal_length_x, focal_length_y, near, far);

  cv::Mat image, depth, mask;
  for (size_t i = 0; i < 100; ++i)
  {
    display.display();
    display.render(image, depth, mask);
    cv::imwrite(boost::str(boost::format("depth_%05d.png") % (i)), depth);
    cv::imwrite(boost::str(boost::format("image_%05d.png") % (i)), image);
    cv::imwrite(boost::str(boost::format("mask_%05d.png") % (i)), mask);
  }

  return 0;
}
