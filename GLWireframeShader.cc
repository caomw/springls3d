/*
 * GLWireframeShader.cc
 *
 *  Created on: Sep 28, 2014
 *      Author: blake
 */

#include "GLWireframeShader.h"

namespace imagesci {

GLWireframeShader::GLWireframeShader():GLShader() {
	// TODO Auto-generated constructor stub

}
void GLWireframeShader::begin(){
	glUseProgram(GetProgramHandle());
}
void GLWireframeShader::end(){
	glUseProgram((GLuint)NULL);
}
bool GLWireframeShader::Init(){
	std::vector<std::string> attrib={"vp","vn"};
	return Initialize(ReadTextFile("wireframe.vert"),ReadTextFile("wireframe.frag"),ReadTextFile("wireframe.geom"),attrib);
}
GLWireframeShader::~GLWireframeShader() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
