/*
 * Copyright Regents of the University of Minnesota, 2014.  This software is released under the following license: http://opensource.org/licenses/lgpl-3.0.html.
 * Source code originally developed at the University of Minnesota Interactive Visualization Lab (http://ivlab.cs.umn.edu).
 *
 * Code author(s):
 * 		Dan Orban (dtorban)
 */

#ifndef EXAMPLEVRAPP_H_
#define EXAMPLEVRAPP_H_

#define GLM_SWIZZLE

#include "GL/glew.h"
#include "MVRCore/AbstractMVRApp.H"
#include "MVRCore/AbstractCamera.H"
#include "MVRCore/AbstractWindow.H"

#include <glm/glm.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>


#include "MVRCore/Event.H"
#include <GLFW/glfw3.h>
#include <boost/thread.hpp>
#include <vector>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/DefaultLogger.hpp>
#include <assimp/LogStream.hpp>

#include <coordinateFrame.h>
#include <gesture.h>
#include <constrainer.h>
#include <finePoint.h>
#include <extraMath.h>

class ExampleVrApp : public MinVR::AbstractMVRApp {
public:
	ExampleVrApp();
	virtual ~ExampleVrApp();

	void doUserInputAndPreDrawComputation(const std::vector<MinVR::EventRef> &events, double synchronizedTime);
	void initializeContextSpecificVars(int threadId, MinVR::WindowRef window);
	void postInitialization();
	void drawGraphics(int threadId, MinVR::AbstractCameraRef camera, MinVR::WindowRef window);
	void getAsset(const std::string &filename, double uniformScale, bool flatShade);
	void dockingTaskHandler(glm::dvec4 grabOffset);
	void defaultTaskHandler();
	void taskHandler(std::string state, bool isStateChanged);
	glm::dvec4 setGrabOffset(std::string state);
	void updateCursorState(glm::dvec4 penOffsetVec);
	void updateShadow();
	void testFunction(void);

private:
	void initGL();
	void initLights();
	//void initVBO_mesh(bool hasNorms, bool hasColors, std::vector<glm::dvec3> &verts, std::vector<glm::dvec3> &norms, std::vector<glm::dvec3> &vertCols);
	void initVBO(bool hasNorms, bool hasColors, bool hasTexCoords, std::vector<int> &indices, std::vector<glm::dvec3> &verts, std::vector<glm::dvec3> &norms, std::vector<glm::dvec3> &vertCols, std::vector<glm::dvec2> &texCoords);
	boost::thread_specific_ptr<GLuint> _vboId;
	boost::thread_specific_ptr<GLuint> _vaoId;
	boost::thread_specific_ptr<GLuint> _eboId;
	boost::thread_specific_ptr<GLuint> _tboId;
	boost::thread_specific_ptr<GLuint> _texId;
	//boost::thread_specific_ptr<GLuint> _lineVboId;
	//glm::dmat4 _mat;
	void moveObject(glm::dvec4 cursorOffsetVec, glm::dvec4 penOffsetVec);
	void grabObject();
	bool selectObject();
	void releaseObject(glm::dvec4 releaseOffsetVec);
	void computeDockingError();	

	float grabRadius;
	float scaleFactor;
	float rotateScale;
	float translateScale;
	bool enableTextures;
	glm::dvec4 penCenter;

	glm::dmat4 _dock;
	glm::dmat4 _dockShadow;	

	FinePoint fp;

	bool grabButtonDown;
	bool graspStart;
	bool grabButtonUp;
	bool isRKeyHeld;
	bool isTKeyHeld;

	bool isGrabbed;
	bool reduceRotate;
	bool reduceTranslate;
	bool drawDock;

	//bool isShadow;

	std::string mode; //number keys: 1, 2, 3... etc
	std::string tool; //keyboard keys correspond to the tool.

	glm::dmat4 _object; 
	//"cursor state" are these variables:
	glm::dmat4 _cursor;
	glm::dmat4 _object_cursorframe;
	glm::dmat4 _grabCursor;
	glm::dmat4 _lastCursor;
	glm::dvec4 cursorOffsetVec;

	float floor_y;
	float floor_width;
	glm::dmat4 _shadow;

	int numIndices, numVerts, numNorms, numVertCols, numTexCoords;
	GLuint vertLoc, normLoc, colorLoc;
	CoordinateFrame cursorFrame;
	CoordinateFrame objectFrame;
};
#endif /* EXAMPLEVRAPP_H_ */
