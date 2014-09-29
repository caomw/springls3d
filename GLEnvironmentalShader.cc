/*
 * GLEnvironmentalShader.cc
 *
 *  Created on: Sep 28, 2014
 *      Author: blake
 */

#include "GLEnvironmentalShader.h"
#include "ImageSciUtil.h"
#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <string>
#include <list>
namespace imagesci {

GLEnvironmentalShader::GLEnvironmentalShader():GLShader(),mTextureId(0) {
	// TODO Auto-generated constructor stub

}
bool GLEnvironmentalShader::Init(const std::string& matcapFile){
	std::vector<RGBA> tmp1;
	int iW,iH;
	if(ReadImageFromFile(matcapFile,tmp1,iW,iH)){
		glGenTextures(1, &mTextureId);
		glBindTexture( GL_TEXTURE_2D, mTextureId);
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, iW, iH, 0, GL_RGBA,GL_UNSIGNED_BYTE, &tmp1[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture( GL_TEXTURE_2D, 0);
	} else {
		std::cerr<<"Could not read "<<matcapFile<<std::endl;
		return false;

	}
	std::list<std::string> attrib;
	attrib.push_back("vp");
	attrib.push_back("vn");
	return Initialize(ReadTextFile("env_shader.vert"),ReadTextFile("env_shader.frag"),"",attrib);
}
void GLEnvironmentalShader::begin(){
	glUseProgram(GetProgramHandle());
	glActiveTexture(GL_TEXTURE0);
	glBindTexture( GL_TEXTURE_2D, mTextureId);

	glUniform1i(glGetUniformLocation(GetProgramHandle(),"matcapTexture"),0);
	glDisable(GL_BLEND);
}
void GLEnvironmentalShader::end(){
	glBindTexture(GL_TEXTURE_2D,0);
	glUseProgram(0);
}
GLEnvironmentalShader::~GLEnvironmentalShader() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */