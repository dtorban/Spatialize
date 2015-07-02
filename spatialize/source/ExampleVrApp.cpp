/*
 * Copyright Regents of the University of Minnesota, 2014.  This software is released under the following license: http://opensource.org/licenses/lgpl-3.0.html.
 * Source code originally developed at the University of Minnesota Interactive Visualization Lab (http://ivlab.cs.umn.edu).
 *
 * Code author(s):
 * 		Dan Orban (dtorban)
 */

#include <example/ExampleVrApp.h>

using namespace MinVR;



ExampleVrApp::ExampleVrApp() : MinVR::AbstractMVRApp(), fp(){
}

ExampleVrApp::~ExampleVrApp() {
	glDeleteBuffersARB(1, _vboId.get());
	glDeleteBuffersARB(1, _eboId.get());
	glDeleteVertexArrays(1, _vaoId.get());
}

void ExampleVrApp::doUserInputAndPreDrawComputation(const std::vector<MinVR::EventRef> &events, double synchronizedTime)
{
	bool isStateChanged = false;
	graspStart = false;
	grabButtonUp = false;
	//set appropriate this-cycle event flags AND set state variables.
	for(int i=0; i < events.size(); i++) {
		if (events[i]->getName() == "WandGrab_Tracker")//"Head_Tracker2")//
		{
			_cursor = events[i]->getCoordinateFrameData();
		}
		else if(events[i]->getName() == "WandGrab_btn_down"){
			graspStart = true;
			grabButtonDown = true;
		}
		else if(events[i]->getName() == "WandGrab_btn_up"){
			grabButtonUp = true;
			grabButtonDown = false;
		}
		/*else if(events[i]->getName() == "kbd_X_up"){
			tool = "x_plane";
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_Z_up"){
			tool = "z_plane";
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_C_up"){
			tool = "y_plane";
			isStateChanged = true;
		}
		*/
		else if(events[i]->getName() == "kbd_R_down"){
			isRKeyHeld = true;
			isStateChanged = true;
			//std::cout << "tool = rotate_only" << std::endl;
			//tool = "rotate_only";
		}
		else if(events[i]->getName() == "kbd_R_up"){
			isRKeyHeld = false;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_T_down"){
			isTKeyHeld = true;
			isStateChanged = true;
			//std::cout << "tool = translate_only" << std::endl;
			//tool = "translate_only";
		}
		else if(events[i]->getName() == "kbd_T_up"){
			isTKeyHeld = false;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_D_up"){
			std::cout << "tool = default" << std::endl;
			tool = "default";
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_A_up")
		{
			tool = "pen_axis";
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_RIGHT_SHIFT_up"){
			reduceRotate = !reduceRotate;
			std::cout << "reduce rotate toggled" << std::endl;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_RIGHT_CONTROL_up"){
			reduceTranslate = !reduceTranslate;
			std::cout << "reduce translate toggled" << std::endl;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_1_up")
		{
			mode = "default";
			drawDock = false;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_2_up")
		{
			mode = "docking_tip";
			drawDock = true;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_3_up")
		{
			mode = "docking_center";
			drawDock = true;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_4_up"){
			mode = "select";
			drawDock = false;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_5_up"){
			mode = "winchHandler";
			drawDock = false;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_6_up"){
			mode = "stateMachine";
			drawDock = false;
			isStateChanged = true;
		}
		else if(events[i]->getName() == "kbd_Y_down"){
			testFunction();
		}
	}
	//std::cout << mode << std::endl;
	glm::dmat3 _cursor2 = glm::dmat3(_cursor[0].xyz, _cursor[1].xyz, _cursor[2].xyz);
	//glm::dvec4 rotatedCenter = _cursor * penCenter;

	if(mode == "stateMachine"){
	
		cursorFrame = CoordinateFrame(_cursor);

		fp.interpretPenInput(cursorFrame, grabButtonDown, isRKeyHeld, isTKeyHeld);
		objectFrame = fp.getObject();
		_object = objectFrame.getMatrix();
		updateShadow();
	}

	else{
			taskHandler(mode, isStateChanged);
		}
	//_lastObject = _object;
}

void
ExampleVrApp::testFunction(){
	CoordinateFrame cFrame = CoordinateFrame(_cursor);
	/*
	glm::dvec4 testVec4 = glm::dvec4(0,0,-1.0, 1);
	glm::dvec4 testVec4_m = _cursor * testVec4;
	glm::dvec3 testVec3 = testVec4.xyz;
	glm::dvec3 testVec3_q = cFrame.getRotation() * testVec3 * glm::inverse(cFrame.getRotation());
	glm::dvec4 testVec4_q = cFrame.getRotation() * testVec4 * glm::inverse(cFrame.getRotation());
	glm::dvec3 testVec3_r = glm::rotate(cFrame.getRotation(), testVec3);
	glm::dvec4 testVec4_r = glm::rotate(cFrame.getRotation(), testVec4);
	std::cout << "diff " << _cursor - cFrame.getMatrix() << std::endl;
	std::cout << "testVec4 " << testVec4 << std::endl;
	std::cout << "testVec3 " << testVec3 << std::endl;
	std::cout << "testVec4_m " << testVec4_m << std::endl;
	std::cout << "testVec3_q " << testVec3_q << std::endl;
	std::cout << "testVec4_q " << testVec4_q << std::endl;
	std::cout << "testVec3_r " << testVec3_r << std::endl;
	std::cout << "testVec4_r " << testVec3_r << std::endl;
	*/
	//glm::dvec4 testVec4_m_i = glm::inverse(_cursor) * testVec4;
	//glm::dvec3 testVec3_q_i = glm::inverse(cFrame.getRotation()) * testVec3 * cFrame.getRotation();
	//glm::dvec4 testVec4_q_i = glm::inverse(cFrame.getRotation()) * testVec4 * cFrame.getRotation();

	//std::cout << "testVec4_m_i " << testVec4_m_i << std::endl;
	//std::cout << "testVec3_q_i " << testVec3_q_i << std::endl;
	//std::cout << "testVec4_q_i " << testVec4_q_i << std::endl;
	
	glm::dquat p = glm::normalize(glm::dquat(1, 2, 3, 4));
	glm::dquat q = glm::normalize(glm::dquat(3, 1, 4, 2));
	glm::dquat c = glm::normalize(cFrame.getRotation());

	glm::dquat multQuat = p * q;
	glm::dvec4 multVec = glm::dvec4(multQuat.x, multQuat.y, multQuat.z, multQuat.w);
	std::cout << "p*q: " << multVec << std::endl;

	glm::dvec3 cVec = glm::axis(c);
	double cAngle = glm::angle(c);
	glm::dvec3 rVec = glm::rotate(p, cVec);

	glm::dquat quat = p * c * glm::inverse(p);
	glm::dvec3 qVec = glm::axis(quat);
	std::cout << "rVec: " << rVec << std::endl;
	std::cout << "qVec: " << qVec << std::endl;

	/*
	glm::dvec3 p_vec = glm::dvec3(p.x, p.y, p.z);
	glm::dvec3 q_vec = glm::dvec3(q.x, q.y, q.z);

	glm::dquat hamilton;
	double mQ2_s = p.w*q.w - glm::dot(p_vec, q_vec);
	glm::dvec3 mQ2_vec = (p.w*q_vec) + (q.w*p_vec) + glm::cross(p_vec, q_vec);
	hamilton = glm::dquat(mQ2_s, mQ2_vec);
	std::cout << "hamilton: " << mQ2_s << " " << mQ2_vec << std::endl;
	*/
}

void
ExampleVrApp::taskHandler(std::string mode, bool isStateChanged){

	glm::dvec4 grabOffset = setGrabOffset(mode);
	bool justGrabbed = false;
	if (!isGrabbed && grabButtonUp){
		if(mode != "select" || selectObject()){
			grabObject();
			justGrabbed = true;
			isStateChanged = true;
		}
	}
	if(isStateChanged){
		updateCursorState(grabOffset);
	}

	if(isGrabbed && grabButtonUp && !justGrabbed){
		computeDockingError();
		releaseObject(glm::dvec4(0.0));
	}
	else if (isGrabbed){
		moveObject(cursorOffsetVec, grabOffset);
	}
	_lastCursor = _cursor;
}

glm::dvec4
ExampleVrApp::setGrabOffset(std::string mode){
	if(mode == "docking_tip"){
		return glm::dvec4(0,0,0,1);
	}
	else if(mode == "docking_center" || mode == "select" || mode == "default"){
		return penCenter;
	}
	else return penCenter;
}

bool
ExampleVrApp::selectObject(){
	bool grabbed = false;
	glm::dvec4 rayCast = _cursor * glm::dvec4(0,0,-1,0);
	glm::dvec4 pos;
	glm::dvec4 norm;
	glm::dvec4 rayOrigin = _cursor[3];
	grabbed = glm::intersectRaySphere(rayOrigin, rayCast, _object[3], grabRadius, pos, norm);
	return grabbed;
}
void
ExampleVrApp::grabObject(){
	std::cout << "Grabbed!" << std::endl;
	isGrabbed = true;
			
}

void
ExampleVrApp::updateCursorState(glm::dvec4 penOffsetVec){
	glm::dvec4 grabCenter = _cursor * penOffsetVec;
	cursorOffsetVec = _object[3] - grabCenter; //actually the offset from the center of the pen.
	_object_cursorframe = glm::inverse(_cursor) * _object;
	_grabCursor = _cursor;
}

void
ExampleVrApp::moveObject(glm::dvec4 cursorOffsetVec, glm::dvec4 penOffsetVec){
	//movement while grabbed
	glm::dmat4 _lastObject = _object;
	glm::dmat4 _storeCursor = _cursor;
	//This doesn't work and I don't know why. I think part of it is just, too many calculations bogging down the system so it gets jittery.
	/*if(reduceRotate){
		glm::dmat3 cursor3 = glm::dmat3(_cursor[0].xyz, _cursor[1].xyz, _cursor[2].xyz);
		glm::dmat3 lastCursor3 = glm::dmat3(_lastCursor[0].xyz, _lastCursor[1].xyz, _lastCursor[2].xyz);
		glm::dmat3 rot = glm::transpose(cursor3) * lastCursor3;
		glm::dvec3 rotDiff = glm::dvec3(glm::atan(rot[1].z, rot[2].z), -1*glm::asin(rot[0].z), glm::atan(rot[0].y, rot[0].x));
		std::cout << rotDiff << std::endl;
		rotDiff.x *= rotateScale;
		rotDiff.y *= rotateScale;
		rotDiff.z *= rotateScale;

		double theta = rotDiff.y;
		double phi = rotDiff.x;
		double psi = rotDiff.z;
		double costheta = glm::cos(theta);
		double sintheta = glm::sin(theta);
		double cosphi = glm::cos(phi);
		double sinphi = glm::sin(phi);
		double cospsi = glm::cos(psi);
		double sinpsi = glm::sin(psi);
		glm::dmat4 rot4 = glm::dmat4(1.0f);
		rot4[0] = glm::dvec4(costheta * cospsi, -1*costheta*sinpsi, sintheta, _cursor[0].w);
		rot4[1] = glm::dvec4(cosphi*sinpsi + sinphi*sintheta*cospsi, cosphi*cospsi - sinphi*sintheta*sinpsi, -1*sinphi*costheta, _cursor[1].w);
		rot4[2] = glm::dvec4(sinphi*sinpsi - cosphi*sintheta*cospsi, sinphi*cospsi + cosphi*sintheta*sinpsi, cosphi*costheta, _cursor[2].w);
		//std::cout << rot << std::endl;
		_cursor = rot4 * _lastCursor;
		_cursor[3] = _storeCursor[3];
		//std::cout << rot4 << std::endl;
	}
	*/
	//TODO : optimize this! Store lastQuat as a variable, and store the current quat in last quat after it finishes.
	if(reduceRotate){
		glm::dmat4 cursor4 = _cursor;
		cursor4[3] = glm::dvec4(0,0,0,1);
		glm::dmat4 lastCursor4 = _lastCursor;
		lastCursor4[3] = glm::dvec4(0,0,0,1);
		glm::dquat lastQuat = glm::quat_cast(lastCursor4);
		glm::dquat quat = glm::quat_cast(cursor4);
		glm::dquat slerpQuat = glm::slerp(lastQuat, quat, 0.1);
		cursor4 = glm::mat4_cast(slerpQuat);
		cursor4[3] = _cursor[3];
		_cursor = cursor4;

	}
	if(reduceTranslate){
		glm::dvec4 translateVec = _storeCursor[3] - _grabCursor[3];
		translateVec = translateScale*translateVec;
		translateVec.w = 0;
		_cursor[3] = translateVec + _grabCursor[3];
	}
	if(tool == "rotate_only"){
		_object = _cursor * _object_cursorframe;
		_object[3] = _lastObject[3]; //add offset between cursor location and object location
	}
	else if (tool == "translate_only"){
		glm::dvec4 grabCenter = _cursor * penOffsetVec; //offset by 
		_object[3] = grabCenter + cursorOffsetVec; //add offset between cursor location and object location
	}
	else if (tool == "pen_axis"){
		glm::dmat4 _tempCursor = glm::inverse(_grabCursor) * _cursor;
		glm::dvec4 axisOnlyVec = glm::dvec4(0,0, _tempCursor[3].z, _tempCursor[3].w);
		axisOnlyVec += penOffsetVec;
		axisOnlyVec.w = 1;
		axisOnlyVec = _grabCursor * axisOnlyVec;
		_object = _cursor * _object_cursorframe;
		_object[3] = axisOnlyVec + cursorOffsetVec; //add offset between cursor location and object location
	}
	else {
		_object = _cursor * _object_cursorframe;
		glm::dvec4 grabCenter = _cursor * penOffsetVec; //offset by 
		_object[3] = grabCenter + cursorOffsetVec; //add offset between cursor location and object location
	}
	_cursor = _storeCursor;
	updateShadow();
}



void
ExampleVrApp::updateShadow(){
	_shadow = _object;
	_shadow[0].y = 0;
	_shadow[1].y = 0;
	_shadow[2].y = 0;
	//std::cout << _object << std::endl;
	_shadow[3].y = floor_y + 0.01;
}

void
ExampleVrApp::releaseObject(glm::dvec4 releaseOffsetVec){

	std::cout << "Released!" << std::endl;
	isGrabbed = false;
	_object[3] += releaseOffsetVec;
}



void
ExampleVrApp::computeDockingError(){
	glm::dvec3 translateError =  _object[3].xyz - _dock[3].xyz;
	std::cout<< "Translate Error: " << translateError << std::endl;
	glm::dmat4 temp_obj = _object;
	temp_obj[3].xyz = _dock[3].xyz;
	glm::dmat4 rot = _dock * glm::inverse(temp_obj);
	
	//Rotational error measured in Tait-Bryan angles.
	glm::dvec3 rotateError = glm::dvec3(glm::atan(rot[1].z, rot[2].z), -1*glm::asin(rot[0].z), glm::atan(rot[0].y, rot[0].x));
	std::cout<< "Rotate Error: " << rotateError << std::endl;
}



void ExampleVrApp::initializeContextSpecificVars(int threadId,
		MinVR::WindowRef window) {
	initGL();
	initLights();
	glClearColor(0.f, 0.3f, 1.f, 1.f);
	_object = glm::dmat4(1.0f);
	
	isRKeyHeld = false;
	isTKeyHeld = false;
	Gesture::initialize();
	objectFrame = CoordinateFrame(_object);
	cursorFrame = CoordinateFrame(_cursor);
	//_object = glm::scale(_object, glm::dvec3(0.1f, 0.1f, 0.1f));
	_object_cursorframe = glm::dmat4(1.0f);

	floor_y = -0.5f;
	floor_width = 2.0f;

	_shadow = glm::dmat4(1.0f);
	_shadow[0].y = 0;
	_shadow[1].y = 0;
	_shadow[2].y = 0;
	_shadow[3].y = floor_y + 0.01;

	_dock = glm::dmat4(1.0f);
	_dockShadow = glm::scale(_dock, glm::dvec3(1.0f, 0.0f, 1.0f));
	_dockShadow[3].y = floor_y + 0.01;

	
	//isShadow = true;
	rotateScale = 0.5;
	translateScale = 0.5;
	reduceRotate = false;
	reduceTranslate = false;

	mode = "stateMachine";
	drawDock = false;
	vertLoc = 0;
	normLoc = 1;
	colorLoc = 2;
	enableTextures = false;
	penCenter = glm::dvec4(0.0f, 0.0f, 0.22f, 1.0f); 
	//the natural place we think is the midpoint is slightly off from the actual midpoint, so we set it to .22 instead of .25 to try to approximate the natural location
	scaleFactor = 0.003f;
	grabRadius = 0.1f;

	getAsset("C:/vis/sw/user-checkouts/torze004/src/ExampleVR/teapot.obj", scaleFactor, false);

	grabButtonUp = false;
	isGrabbed = false;
	grabButtonDown = false;
	GLenum err;
	if((err = glGetError()) != GL_NO_ERROR) {
		std::cout << "openGL ERROR in initializeContextSpecificVars: "<<err<<std::endl;
	}
}



void
ExampleVrApp::getAsset(const std::string &filename, double uniformScale, bool flatShade)
{
	Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
	// Create a logger instance for Console Output
	Assimp::DefaultLogger::create("",severity, aiDefaultLogStream_STDOUT);
	Assimp::Importer* importer = new Assimp::Importer();

	const aiScene* scene = importer->ReadFile(filename, aiProcess_JoinIdenticalVertices | aiProcess_Triangulate | aiProcess_GenNormals);

	//G3D::Array<ThreeTexturedMeshRef> meshes;
	std::vector<glm::vec3> meshes;
	// If the import failed, report it
	if( !scene) {
		std::cout << "scene not found!\n" << std::endl;
		Assimp::DefaultLogger::get()->info(importer->GetErrorString());
		delete importer;
		Assimp::DefaultLogger::kill();
		//return meshes;
	}

	if(scene->mNumMeshes >= 1)
	{
		//for (int f = 0; f < scene->mNumMeshes; f++)
		for (int f = 0; f < 1; f++) //ONLY MAKES FIRST MESH.
		{
			const aiMesh* mesh = scene->mMeshes[f];
			const aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
			

			if (mesh->mPrimitiveTypes == aiPrimitiveType_TRIANGLE) {
				std::vector<int> indices;
				std::vector<glm::dvec3> verts;
				std::vector<glm::dvec3> norms;
				std::vector<glm::dvec3> vertCols;
				std::vector<glm::dvec2> texCoords;


				bool hasColors = mesh->HasVertexColors(0);
				bool hasNormals = mesh->HasNormals();
				bool hasTexCoords = mesh->HasTextureCoords(0);
				
				aiColor3D matColor (0.f,0.f,0.f);
				bool hasMatColor = mesh->mMaterialIndex > 0 && material->Get(AI_MATKEY_COLOR_DIFFUSE,matColor) == AI_SUCCESS;

				if(!flatShade) {
					//copy the vert array
					for(int v = 0; v < mesh->mNumVertices; v++) {
						aiVector3D position = mesh->mVertices[v];
						verts.push_back (uniformScale*glm::dvec3(position.x, position.y, position.z));
						if(hasNormals) {
							aiVector3D normal = mesh->mNormals[v];
							norms.push_back(glm::dvec3(normal.x,normal.y,normal.x));
						}
						if(hasColors || hasMatColor) {
							if (hasMatColor)
							{
								vertCols.push_back(glm::dvec3(matColor.r,matColor.g,matColor.b));
							}
							else
							{
								aiColor4D color = mesh->mColors[0][v];
								vertCols.push_back(glm::dvec3(color.r,color.g,color.b));
							}
						}
						if(hasTexCoords) {
							texCoords.push_back(glm::dvec2(mesh->mTextureCoords[0][v].x, mesh->mTextureCoords[0][v].y));
						}
					}
					//copy the index array
					for(int f = 0; f < mesh->mNumFaces; f++) {
						for(int v = 0; v < 3; v++) {
							int index = mesh->mFaces[f].mIndices[v];
							indices.push_back(index);
						}
					}
				}
				else {
					int count = 0;
					for(int f = 0; f < mesh->mNumFaces; f++) {
						for(int v = 0; v < 3; v++) {
							int index = mesh->mFaces[f].mIndices[v];
							aiVector3D position = mesh->mVertices[index];
							verts.push_back(uniformScale*glm::dvec3(position.x,position.y,position.z));
							if(hasNormals) {
								aiVector3D normal = mesh->mNormals[index];
								norms.push_back(glm::dvec3(normal.x,normal.y,normal.x));
							}
							if(hasColors) {
								aiColor4D color = mesh->mColors[0][index];
								vertCols.push_back(glm::dvec3(color.r,color.g,color.b));
							}
							if(hasTexCoords) {
								texCoords.push_back(glm::dvec2(mesh->mTextureCoords[0][v].x, mesh->mTextureCoords[0][v].y));
							}
							indices.push_back(count);
							count++;
						}
					}
				}
				initVBO(hasNormals, hasColors, hasTexCoords, indices, verts, norms, vertCols, texCoords);
				numIndices = indices.size();
				numVerts = verts.size();
				numNorms = norms.size();
				numTexCoords = texCoords.size();
				numVertCols = vertCols.size();

			}


		}
	}

	delete importer;
	Assimp::DefaultLogger::kill();
}

/* Incomplete: no support for textures yet.
 * 
 *
*/
void ExampleVrApp::initVBO(bool hasNormals, bool hasColors, bool hasTexCoords, std::vector<int> &indices, std::vector<glm::dvec3> &verts, std::vector<glm::dvec3> &norms, std::vector<glm::dvec3> &vertCols, std::vector<glm::dvec2> &texCoords){
	_vboId.reset(new GLuint(0));
	_vaoId.reset(new GLuint(0));
	_eboId.reset(new GLuint(0));
	if(hasTexCoords){
		_tboId.reset(new GLuint(0));
		_texId.reset(new GLuint(0));
	}
	glGenVertexArrays(1, _vaoId.get());
	glBindVertexArray(*(_vaoId.get()));

	glGenBuffers(1, _vboId.get());
	glGenBuffers(1, _eboId.get());
	glBindBuffer(GL_ARRAY_BUFFER, *(_vboId.get()));
	
	glBufferData(GL_ARRAY_BUFFER, 
		(sizeof(glm::dvec3) * verts.size())
		+ (sizeof(glm::dvec3) * norms.size())
		+ (sizeof(glm::dvec3) * verts.size()), 
		0, GL_STATIC_DRAW);
	
	glBufferSubData(GL_ARRAY_BUFFER, 
		0, 
		(sizeof(glm::dvec3)) * verts.size(), 
		&verts[0]);

	if(hasNormals){
	glBufferSubData(GL_ARRAY_BUFFER, 
		sizeof(glm::dvec3) * verts.size(), 
		sizeof(glm::dvec3) * norms.size(),
		&norms[0]);
	}
	if(hasColors){
	glBufferSubData(GL_ARRAY_BUFFER, 
		sizeof(glm::dvec3) * verts.size() + sizeof(glm::dvec3) * norms.size(), 
		sizeof(glm::dvec3) * vertCols.size(),
		&vertCols[0]);
	}
	
	glEnableVertexAttribArray(vertLoc);
	glVertexAttribPointer(vertLoc, 3, GL_DOUBLE, GL_FALSE, 0, (char*)NULL + 0); //verts
	
	if(hasNormals){
		glEnableVertexAttribArray(normLoc);
		glVertexAttribPointer(normLoc, 3, GL_DOUBLE, GL_FALSE, 0, (char*)NULL + (sizeof(double)*3*verts.size())); //norms
		
	}
	if(hasColors){
		glEnableVertexAttribArray(colorLoc);
		glVertexAttribPointer(colorLoc, 3, GL_DOUBLE, GL_FALSE, 0, (char*)NULL + (sizeof(double)*3*(verts.size() + norms.size())));
		
	}
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *(_eboId.get()));
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	if(hasTexCoords){
		glBindBuffer(GL_TEXTURE_BUFFER, *(_tboId.get()));
		glBufferData(GL_TEXTURE_BUFFER, texCoords.size() * sizeof(glm::dvec2), &texCoords[0], GL_STATIC_DRAW);
		glBindBuffer(GL_TEXTURE_BUFFER, 0);
	}
	GLenum err;
	if((err = glGetError()) != GL_NO_ERROR) {
		std::cout << "GLERROR initVBO_mesh: "<<err<<std::endl;
	}
}

void ExampleVrApp::initGL()
{
	glShadeModel(GL_SMOOTH);                    // shading mathod: GL_SMOOTH or GL_FLAT
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);      // 4-byte pixel alignment

    // enable /disable features
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    //glEnable(GL_CULL_FACE);
     // track material ambient and diffuse from surface color, call it before glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glClearColor(0, 0, 0, 0);                   // background color
    glClearStencil(0);                          // clear stencil buffer
    glClearDepth(1.0f);                         // 0 is near, 1 is far
    glDepthFunc(GL_LEQUAL);
	
	GLenum err;
	if((err = glGetError()) != GL_NO_ERROR) {
		std::cout << "GLERROR initGL: "<<err<<std::endl;
	}
}

void ExampleVrApp::initLights()
{
	// set up light colors (ambient, diffuse, specular)
    GLfloat lightKa[] = {.2f, .2f, .2f, 1.0f};  // ambient light
    GLfloat lightKd[] = {.7f, .7f, .7f, 1.0f};  // diffuse light
    GLfloat lightKs[] = {1, 1, 1, 1};           // specular light
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightKa);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightKd);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightKs);

    // position the light
    float lightPos[4] = {0.5, 0, 3, 1}; // positional light
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    glEnable(GL_LIGHT0);                        // MUST enable each light source after configuration
	glShadeModel (GL_SMOOTH);
	GLenum err;
	if((err = glGetError()) != GL_NO_ERROR) {
		std::cout << "GLERROR initLights: "<<err<<std::endl;
	}
}

void ExampleVrApp::postInitialization() {
}

void ExampleVrApp::drawGraphics(int threadId, MinVR::AbstractCameraRef camera,
		MinVR::WindowRef window) {

	GLenum err;
	while((err = glGetError()) != GL_NO_ERROR) {
		std::cout << "GLERROR: "<<err<<std::endl;
	}

	glBindBuffer(GL_ARRAY_BUFFER, *(_vboId.get()));

    // enable vertex arrays
	if(numNorms > 0){
		glEnableClientState(GL_NORMAL_ARRAY);
	}
	if(numVertCols > 0){
		glEnableClientState(GL_COLOR_ARRAY);
	}
	glEnableClientState(GL_VERTEX_ARRAY);

	if(numTexCoords > 0 && enableTextures){
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	}
    // before draw, specify vertex and index arrays with their offsets
    if(numNorms > 0){
		glNormalPointer(GL_DOUBLE, 0, (void*)(sizeof(glm::dvec3)*numVerts));
	}
	if(numVertCols > 0){
		glColorPointer(3, GL_DOUBLE, 0, (void*)(sizeof(glm::dvec3)*numNorms+ sizeof(glm::dvec3)*numVerts));
	}
	glVertexPointer(3, GL_DOUBLE, 0, 0);

	if(numTexCoords > 0 && enableTextures){

	}
	
	//Index Buffer object
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *(_eboId.get()));

	fp.draw(threadId, camera, window);
	//object
	camera->setObjectToWorldMatrix(_object);
	glColor3f(1.f, 1.f, 1.f);
	glDrawElements(
		GL_TRIANGLES,
		numIndices,
		GL_UNSIGNED_INT,
		(void*)0
	);

	//dock
	glColor3f(1.f, 1.f, 0.5f);
	if(drawDock){
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		camera->setObjectToWorldMatrix(_dock);
		glDrawElements(
			GL_TRIANGLES,
			numIndices,
			GL_UNSIGNED_INT,
			(void*)0
		);
		//dock shadow
		camera->setObjectToWorldMatrix(_dockShadow);
		glColor3f(0.2f, 0.2f, 0.f);
		glDrawElements(
			GL_TRIANGLES,
			numIndices,
			GL_UNSIGNED_INT,
			(void*)0
		);
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	}
	//shadow
	camera->setObjectToWorldMatrix(_shadow);
	glColor3f(0.f, 0.f, 0.f);
	glDrawElements(
		GL_TRIANGLES,
		numIndices,
		GL_UNSIGNED_INT,
		(void*)0
	);

	if(grabButtonDown){
		camera->setObjectToWorldMatrix(_cursor);
		glBegin (GL_LINES);
			glVertex3f(0,0,-.01);
			glVertex3f(0,0,-10);
		glEnd ();
	}
	else if(isGrabbed && tool == "pen_axis"){
		camera->setObjectToWorldMatrix(_grabCursor);
		
		glBegin (GL_LINES);
		glColor3f(1.0f, 1.0f, 1.0f);
			glVertex3f(0,0,-.01);
			glVertex3f(0,0,-10);
		glEnd ();
	}

	//draw teapot scaled way down
	/*
	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	glColor3f(1.0f, 1.0f, 1.0f);
	camera->setObjectToWorldMatrix(glm::scale(_cursor,0.1,0.1,0.1));
		glDrawElements(
			GL_TRIANGLES,
			numIndices,
			GL_UNSIGNED_INT,
			(void*)0
		);
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	*/

    glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

	//Origin
	camera->setObjectToWorldMatrix(glm::dmat4(1.0));
	
	glBegin(GL_TRIANGLES);
	float f_x = floor_width * 0.5;
	glColor3f(1.0f, 0.5f, 0.5f);

	glVertex3f(-f_x, floor_y, -f_x);
	glVertex3f(-f_x, floor_y, f_x);
	glVertex3f(f_x, floor_y, -f_x);
	
	glVertex3f(-f_x, floor_y, f_x);
	glVertex3f(f_x, floor_y, f_x);
	glVertex3f(f_x, floor_y, -f_x);

	glColor3f(0.5f, 0.5f, 1.0f);
	glVertex3f(-f_x, floor_y, -f_x);
	glVertex3f(-f_x, -floor_y, -f_x);
	glVertex3f(f_x, -floor_y, -f_x);

	glVertex3f(-f_x, floor_y, -f_x);
	glVertex3f(f_x, floor_y, -f_x);
	glVertex3f(f_x, -floor_y, -f_x);

	glColor3f(0.8f, 0.4f, 0.8f);
	glVertex3f(-f_x, floor_y, -f_x);
	glVertex3f(-f_x, -floor_y, -f_x);
	glVertex3f(-f_x, -floor_y, f_x);

	glVertex3f(-f_x, floor_y, -f_x);
	glVertex3f(-f_x, floor_y, f_x);
	glVertex3f(-f_x, -floor_y, f_x);

	glVertex3f(f_x, floor_y, -f_x);
	glVertex3f(f_x, -floor_y, -f_x);
	glVertex3f(f_x, -floor_y, f_x);

	glVertex3f(f_x, floor_y, -f_x);
	glVertex3f(f_x, floor_y, f_x);
	glVertex3f(f_x, -floor_y, f_x);

	glEnd();
	
}