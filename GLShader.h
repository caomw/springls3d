/*
 * Copyright(C) 2014, Blake Lucas (img.science@gmail.com)
 * All rights reserved.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
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
