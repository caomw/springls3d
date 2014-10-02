/*
 * Shader.h
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
 */

#ifndef GLSHADER_H_
#define GLSHADER_H_


#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include <string>
#include <vector>
namespace imagesci {

// Shader helper structure.
//
// It simply handles shader compilation and linking.
class GLShader {
public:
	// Default constructor.
	GLShader();
	virtual ~GLShader();
	// Initialization function to compile the shader.
	bool Initialize(const std::string& pVertexShaderString,
			const std::string& pFragmentShaderString,
			const std::string& pGeomShaderString,
			std::vector<std::string>& attributes);

	GLShader(const std::string& pVertexShaderString,
			const std::string& pFragmentShaderString,
			const std::string& pGeomShaderString,
			std::vector<std::string>& attributes):GLShader(){
		Initialize(pVertexShaderString,pFragmentShaderString,pGeomShaderString,attributes);
	}
	// Uninitialization function.
	void Uninitialize();
	virtual void begin();
	virtual void end();
	// Returns the program handle.
	inline GLuint GetProgramHandle() const;

private:
	// Vertex shader handle.
	GLuint mVertexShaderHandle;
	// Fragment shader handle.
	GLuint mFragmentShaderHandle;
	// Fragment shader handle.
	GLuint mGeometryShaderHandle;
	// Program handle.
	GLuint mProgramHandle;

};

inline GLuint GLShader::GetProgramHandle() const {
	return mProgramHandle;
}
}
#endif /* SHADER_H_ */
