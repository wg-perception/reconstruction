//
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c), author on http://www.gamedev.net/topic/582240-assimp-drawing-textured-model/
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

#ifndef MODEL_H
#define MODEL_H

#include <SDL/SDL.h>
#include <SDL/SDL_opengl.h>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <FreeImage.h>
#include <vector>

#include <iostream>

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

struct TextureAndPath
{
  GLuint hTexture;
  aiString pathName;
};

class Model
{
private:
  std::vector<TextureAndPath> texturesAndPaths;
  const struct aiScene* scene;

  void
  recursiveTextureLoad(const struct aiScene *sc, const struct aiNode* nd);
  void
  recursive_render(const struct aiScene *sc, const struct aiNode* nd) const;

  void
  get_bounding_box_for_node(const aiNode* nd, aiVector3D* min, aiVector3D* max, aiMatrix4x4* trafo) const;
public:
  Model();
  ~Model();

  void
  LoadModel(const std::string & fileName);
  void
  Draw() const;

  void
  get_bounding_box(aiVector3D* min, aiVector3D* max) const;
};

void
color4_to_float4(const aiColor4D *c, float f[4]);

void
set_float4(float f[4], float a, float b, float c, float d);

void
apply_material(const struct aiMaterial *mtl);

// Can't send color down as a pointer to aiColor4D because AI colors are ABGR.
void
Color4f(const aiColor4D *color);

#endif
