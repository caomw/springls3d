/*
 * Copyright(C) 2014, Blake C. Lucas, Ph.D. (img.science@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
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
	std::vector<std::string> attrib;
	attrib.push_back("vp");
	attrib.push_back("vn");
	return Initialize(ReadTextFile("env_shader.vert"),ReadTextFile("env_shader.frag"),"",attrib);
}
void GLEnvironmentalShader::begin(){
	glUseProgram(GetProgramHandle());
	glActiveTexture(GL_TEXTURE0);
	glBindTexture( GL_TEXTURE_2D, mTextureId);

	glUniform1i(glGetUniformLocation(GetProgramHandle(),"matcapTexture"),0);

}
void GLEnvironmentalShader::end(){
	glBindTexture(GL_TEXTURE_2D,0);
	glUseProgram(0);
}
GLEnvironmentalShader::~GLEnvironmentalShader() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
