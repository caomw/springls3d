/*
 * Shader.h
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
 */

#ifndef GLSHADER_H_
#define GLSHADER_H_
#include <GL/gl.h>


namespace imagesci {

// Shader helper structure.
//
// It simply handles shader compilation and linking.
class GLShader {
public:
	// Default constructor.
	GLShader();

	// Initialization function to compile the shader.
	bool Initialize(const char pVertexShaderString[],
			const char pFragmentShaderString[],
			const char* const pAttributeLocations[]);
	// Uninitialization function.
	void Uninitialize();

	// Returns the program handle.
	inline GLuint GetProgramHandle() const;

private:
	// Vertex shader handle.
	GLuint mVertexShaderHandle;
	// Fragment shader handle.
	GLuint mFragmentShaderHandle;
	// Program handle.
	GLuint mProgramHandle;

};

inline GLuint GLShader::GetProgramHandle() const {
	return mProgramHandle;
}
}
#endif /* SHADER_H_ */
