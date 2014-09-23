/*
 * Shader.cc
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
 */

#include "GLShader.h"
#include <iostream>
#include <GL/glx.h>
#include <GL/glu.h>
namespace imagesci {

GLShader::GLShader() :
		mVertexShaderHandle(0), mFragmentShaderHandle(0), mProgramHandle(0) {
}

bool GLShader::Initialize(const char pVertexShaderString[],
		const char pFragmentShaderString[],
		const char* const pAttributeLocations[]) {
	GLint lStatus;

	// Compile vertex shader.
	mVertexShaderHandle = glCreateShader( GL_VERTEX_SHADER);
	glShaderSource(mVertexShaderHandle, 1, &pVertexShaderString, 0);
	glCompileShader(mVertexShaderHandle);

	// Verify that shader compiled correctly.
	glGetShaderiv(mVertexShaderHandle, GL_COMPILE_STATUS, &lStatus);
	if (lStatus != GL_TRUE) {
		// FIXME: We could add code to check why the shader didn't compile.
		std::cerr << "Unable to compile vertex shader properly..." << std::endl;
		glDeleteShader(mVertexShaderHandle);
		mVertexShaderHandle = 0;
		return false;
	}

	// Compile fragment shader.
	mFragmentShaderHandle = glCreateShader( GL_FRAGMENT_SHADER);
	glShaderSource(mFragmentShaderHandle, 1, &pFragmentShaderString, 0);
	glCompileShader(mFragmentShaderHandle);

	// Verify that shader compiled correctly.
	glGetShaderiv(mFragmentShaderHandle, GL_COMPILE_STATUS, &lStatus);
	if (lStatus != GL_TRUE) {
		// FIXME: We could add code to check why the shader didn't compile.
		std::cerr << "Unable to compile fragment shader properly..."
				<< std::endl;
		glDeleteShader(mVertexShaderHandle);
		mVertexShaderHandle = 0;
		glDeleteShader(mFragmentShaderHandle);
		mFragmentShaderHandle = 0;
		return false;
	}

	// Link shaders.
	mProgramHandle = glCreateProgram();
	glAttachShader(mProgramHandle, mVertexShaderHandle);
	glAttachShader(mProgramHandle, mFragmentShaderHandle);

	// Bind the attribute location for all vertices.
	int lIndex = 0;
	while (pAttributeLocations[lIndex] != 0) {
		glBindAttribLocation(mProgramHandle, lIndex,
				pAttributeLocations[lIndex]);
		++lIndex;
	}

	glLinkProgram(mProgramHandle);

	// Verify that program linked correctly.
	glGetProgramiv(mProgramHandle, GL_LINK_STATUS, &lStatus);
	if (lStatus != GL_TRUE) {
		// FIXME: We could add code to check why the shader didn't link.
		std::cerr << "Unable to link shaders properly..." << std::endl;
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

	glDeleteProgram(mProgramHandle);
	mProgramHandle = 0;
}
} /* namespace imagesci */
