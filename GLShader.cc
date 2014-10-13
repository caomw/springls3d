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
#include "GLShader.h"
#include "ImageSciUtil.h"
#include <iostream>



namespace imagesci {

GLShader::GLShader() :
		mVertexShaderHandle(0), mFragmentShaderHandle(0), mGeometryShaderHandle(0),mProgramHandle(0) {

}
void GLShader::begin(){
	glUseProgram(GetProgramHandle());
}
void GLShader::end(){
	glUseProgram((GLuint)NULL);
}
GLShader::~GLShader(){
	Uninitialize();
}
bool GLShader::Initialize(
		const std::string& pVertexShaderString,
		const std::string& pFragmentShaderString,
		const std::string& pGeometryShaderString,
		std::vector<std::string>& pAttributeLocations) {
	GLint lStatus;
	char message[4096]="";
	// Compile vertex shader.
	mVertexShaderHandle = glCreateShader( GL_VERTEX_SHADER);
	const char* code= pVertexShaderString.c_str();
	glShaderSource(mVertexShaderHandle, 1,&code, 0);
	glCompileShader(mVertexShaderHandle);

	// Verify that shader compiled correctly.
	glGetShaderiv(mVertexShaderHandle, GL_COMPILE_STATUS, &lStatus);
	if (lStatus != GL_TRUE) {
		// FIXME: We could add code to check why the shader didn't compile.
		std::cerr << "Unable to compile vertex shader properly..." << std::endl;
		glGetInfoLogARB(mVertexShaderHandle,sizeof(message),NULL,message);
		std::cerr<<message<<std::endl;

		glDeleteShader(mVertexShaderHandle);
		mVertexShaderHandle = 0;
		glDeleteShader(mFragmentShaderHandle);
		mGeometryShaderHandle = 0;
		glDeleteShader(mGeometryShaderHandle);
		mGeometryShaderHandle = 0;
		return false;
	}

	// Compile fragment shader.
	mFragmentShaderHandle = glCreateShader( GL_FRAGMENT_SHADER);
	code= pFragmentShaderString.c_str();
	glShaderSource(mFragmentShaderHandle, 1, &code, 0);
	glCompileShader(mFragmentShaderHandle);

	// Verify that shader compiled correctly.
	glGetShaderiv(mFragmentShaderHandle, GL_COMPILE_STATUS, &lStatus);
	if (lStatus != GL_TRUE) {
		// FIXME: We could add code to check why the shader didn't compile.
		std::cerr << "Unable to compile fragment shader properly..."
				<< std::endl;
		glGetInfoLogARB(mFragmentShaderHandle,sizeof(message),NULL,message);
		std::cerr<<message<<std::endl;

		glDeleteShader(mVertexShaderHandle);
		mVertexShaderHandle = 0;
		glDeleteShader(mFragmentShaderHandle);
		mGeometryShaderHandle = 0;
		glDeleteShader(mGeometryShaderHandle);
		mGeometryShaderHandle = 0;
		return false;
	}


	if(pGeometryShaderString.length()>0){

		// Compile Geometry shader.
		mGeometryShaderHandle = glCreateShader(GL_GEOMETRY_SHADER);
		code= pGeometryShaderString.c_str();
		glShaderSource(mGeometryShaderHandle, 1,&code, 0);
		glCompileShader(mGeometryShaderHandle);
		// Verify that shader compiled correctly.
		glGetShaderiv(mGeometryShaderHandle, GL_COMPILE_STATUS, &lStatus);
		if (lStatus != GL_TRUE) {
			std::cerr << "Unable to compile Geometry shader properly..."
					<< std::endl;
			glGetInfoLogARB(mGeometryShaderHandle,sizeof(message),NULL,message);
			std::cerr<<message<<std::endl;

			glDeleteShader(mVertexShaderHandle);
			mVertexShaderHandle = 0;
			glDeleteShader(mFragmentShaderHandle);
			mGeometryShaderHandle = 0;
			glDeleteShader(mGeometryShaderHandle);
			mGeometryShaderHandle = 0;
			return false;
		}

	}
	// Link shaders.
	mProgramHandle = glCreateProgram();
	glAttachShader(mProgramHandle, mVertexShaderHandle);
	glAttachShader(mProgramHandle, mFragmentShaderHandle);
	if(mGeometryShaderHandle>0)glAttachShader(mProgramHandle, mGeometryShaderHandle);

	// Bind the attribute location for all vertices.
	int lIndex = 0;
	for(std::string str:pAttributeLocations) {
		glBindAttribLocation(mProgramHandle, lIndex,str.c_str());
		++lIndex;
	}
	glLinkProgram(mProgramHandle);

	// Verify that program linked correctly.
	glGetProgramiv(mProgramHandle, GL_LINK_STATUS, &lStatus);
	if (lStatus != GL_TRUE) {

		// FIXME: We could add code to check why the shader didn't link.
		std::cerr << "Unable to link shaders properly..." << std::endl;
		glGetInfoLogARB(mProgramHandle,sizeof(message),NULL,message);
		std::cerr<<message<<std::endl;
		Uninitialize();
		return false;
	}
	return true;
}

void GLShader::Uninitialize() {
	glDetachShader(mProgramHandle, mFragmentShaderHandle);
	glDeleteShader(mFragmentShaderHandle);
	mFragmentShaderHandle = 0;

	glDetachShader(mProgramHandle, mVertexShaderHandle);
	glDeleteShader(mVertexShaderHandle);
	mVertexShaderHandle = 0;

	glDetachShader(mProgramHandle, mGeometryShaderHandle);
	glDeleteShader(mGeometryShaderHandle);
	mGeometryShaderHandle = 0;
	glDeleteProgram(mProgramHandle);
	mProgramHandle = 0;
}
} /* namespace imagesci */
