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
#ifndef GLENVIRONMENTALSHADER_H_
#define GLENVIRONMENTALSHADER_H_
#include "GLShader.h"
namespace imagesci {

class GLEnvironmentalShader: public GLShader{
protected:
	unsigned int mTextureId;
public:
	GLEnvironmentalShader();
	bool Init(const std::string& matcapFile);
	virtual void begin();
	virtual void end();
	virtual ~GLEnvironmentalShader();
};

} /* namespace imagesci */

#endif /* GLENVIRONMENTALSHADER_H_ */
