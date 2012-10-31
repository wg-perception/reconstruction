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

#include <stdlib.h>

#include <GL/glut.h>

#include "display.h"

// the global Assimp scene object
GLuint scene_list = 0;

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

double PI = 3.14159;

// ----------------------------------------------------------------------------
void
reshape(int width, int height)
{
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity();

  double fx = Display::focal_length_x();
  double fy = Display::focal_length_y();
  double fovy = 2 * atan(0.5 * height / fy) * 180 / PI;
  double aspect = (width * fy) / (height * fx);

  // define the near and far clipping planes
  double near = Display::near();
  double far = Display::far();

  // set perspective
  gluPerspective(fovy, aspect, near, far);
  glViewport(0, 0, width, height);
}

void
normalize_vector(float & x, float&y, float&z)
{
  float norm = std::sqrt(x * x + y * y + z * z);
  x /= norm;
  y /= norm;
  z /= norm;
}
// ----------------------------------------------------------------------------
void
display_function(void)
{
  float tmp;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity();
  //gluLookAt(0.f, 0.f, 2.f, 0.f, 0.f, -5.f, 0.f, 1.f, 0.f);

  // rotate it around the y axis
  //glRotatef(angle, 0.f, 1.f, 0.f);

  // Figure a view point on the sphere
  static unsigned int nx = 15, ny = 15;
  static unsigned int X = 0;
  static unsigned int Y = 0;
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
      ++Y;
      if (Y >= ny/2)
      {
        Y = 0;
        ++X;
        if (X >= nx)
          // Ugly hack to exit glutMainLoop
          throw("something");
      }
    }
  }
  float lon = 2 * PI * ((X + 0.5) / nx);
  float lat = asin((2 * (Y + 0.5) / ny - 1));
  unsigned int n_templates = ((angle_max - angle_min) / angle_step + 1) * nx * ny
                             * ((radius_max - radius_min) / radius_step + 1);
  std::cout << n_templates << " templates: " << float(index * 100) / float(n_templates) << std::endl;
  std::cout << angle << " " << lon << " " << lat << " " << radius << std::endl;

  float x = radius * cos(lon) * sin(lat);
  float y = radius * sin(lon) * sin(lat);
  float z = radius * cos(lat);
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
  Display::model().get_bounding_box(&scene_min, &scene_max);
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
  if (scene_list == 0)
  {
    scene_list = glGenLists(1);
    glNewList(scene_list, GL_COMPILE);
    // now begin at the root node of the imported data and traverse
    // the scenegraph by multiplying subsequent local transforms
    // together on GL's matrix stack.
    Display::model().Draw();
    glEndList();
  }

  glCallList(scene_list);

  glutSwapBuffers();

  if (index > 0)
    Display::save_to_disk();

  glutPostRedisplay();
  ++index;
}

// ----------------------------------------------------------------------------
int
main(int argc, char **argv)
{
  struct aiLogStream stream;

  // Define the display
  size_t width = 640, height = 480;
  double near = 0.1, far = 1000;
  double focal_length_x = 525, focal_length_y = 525;

  // the model name can be specified on the command line.
  Display display;
  Display::load_model(argv[1]);
  Display::set_parameters(width, height, focal_length_x, focal_length_y, near, far);

  glutInitWindowSize(width, height);
  glutInitWindowPosition(100, 100);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInit(&argc, argv);

  glutCreateWindow("Assimp - Very simple OpenGL sample");
  glutDisplayFunc(display_function);
  glutReshapeFunc(reshape);

  // get a handle to the predefined STDOUT log stream and attach
  // it to the logging system. It remains active for all further
  // calls to aiImportFile(Ex) and aiApplyPostProcessing.
  stream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT, NULL);
  aiAttachLogStream(&stream);

  // ... same procedure, but this stream now writes the
  // log messages to assimp_log.txt
  stream = aiGetPredefinedLogStream(aiDefaultLogStream_FILE, "assimp_log.txt");
  aiAttachLogStream(&stream);

  glClearColor(0.1f, 0.1f, 0.1f, 1.f);

  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0); // Uses default lighting parameters

  glEnable (GL_DEPTH_TEST);

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  glEnable (GL_NORMALIZE);

  // XXX docs say all polygons are emitted CCW, but tests show that some aren't.
  if (getenv("MODEL_IS_BROKEN"))
    glFrontFace (GL_CW);

  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

  glutGet(GLUT_ELAPSED_TIME);
  try
  {
    glutMainLoop();
  } catch (const char * msg)
  {

  }

  // We added a log stream to the library, it's our job to disable it
  // again. This will definitely release the last resources allocated
  // by Assimp.
  aiDetachAllLogStreams();
  return 0;
}
