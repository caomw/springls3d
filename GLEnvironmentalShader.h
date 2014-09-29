/*
 * GLEnvironmentalShader.h
 *
 *  Created on: Sep 28, 2014
 *      Author: blake
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
