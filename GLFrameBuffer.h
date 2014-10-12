/*
 * GLFrameBuffer.h
 *
 *  Created on: Oct 1, 2014
 *      Author: blake
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
