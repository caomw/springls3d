/*
 * GLWireframeShader.h
 *
 *  Created on: Sep 28, 2014
 *      Author: blake
 */

#ifndef GLWIREFRAMESHADER_H_
#define GLWIREFRAMESHADER_H_
#include "GLShader.h"
#include "ImageSciUtil.h"

namespace imagesci {

class GLWireframeShader: public GLShader{
public:
	GLWireframeShader();
	virtual void begin();
	virtual void end();
	bool Init();
	virtual ~GLWireframeShader();
};

} /* namespace imagesci */

#endif /* GLWIREFRAMESHADER_H_ */
