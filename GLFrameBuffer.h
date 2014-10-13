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
#ifndef GLFRAMEBUFFER_H_
#define GLFRAMEBUFFER_H_
#include "GLImage.h"
namespace imagesci {

class GLFrameBuffer: public GLImage {
protected:
	unsigned int mFrameBufferId;
	unsigned int mDepthBufferId;
public:
	GLFrameBuffer(int _x,int _y,int _width,int _height,int _imageWidth,int _imageHeight):
	GLImage(_x,_y,_width,_height,_imageWidth,_imageHeight,true),mFrameBufferId(0),mDepthBufferId(0){

	};
	GLFrameBuffer();
	virtual void updateGL();
	void begin();
	void end();
	virtual ~GLFrameBuffer();

};

} /* namespace imagesci */

#endif /* GLFRAMEBUFFER_H_ */
