/*
* Copyright Regents of the University of Minnesota, 2014.  This software is released under the following license: http://opensource.org/licenses/lgpl-3.0.html.
* Source code originally developed at the University of Minnesota Interactive Visualization Lab (http://ivlab.cs.umn.edu).
*
* Code author(s):
* 		Dan Orban (dtorban)
*		Chris Eidsmoe (ceidsmoe)
*/

#ifndef VRMODEL_H_
#define VRMODEL_H_

#include "Scene.h"
#include "Mesh.h"
#include "Shader.h"
#include "GL/glew.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <string>

namespace Spatialize {
	//adapted from code found on learnopengl.com
	class VRModel : public Scene {
	public:
		VRModel(GLchar *path);
		virtual ~VRModel();

		const Box& getBoundingBox();
		void draw(float time, MinVR::CameraRef camera, MinVR::WindowRef window, Shader shader);

	private:
		vector<Texture> textures_loaded;
		vector<Mesh> meshes;
		std::string directory;
		Box _boundingBox;

		void loadModel(std::string path);
		void processNode(aiNode *node, const aiScene *scene);
		Mesh processMesh(aiMesh *mesh, const aiScene *scene);
		vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, std::string typeName);
	};


	GLint TextureFromFile(const char *path, std::string directory);
} /* namespace Spatialize */

#endif /* VRModel_H_ */
