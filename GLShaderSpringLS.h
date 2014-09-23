/*
 * GLShaderSpringLS.h
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
 */

#ifndef GLSHADERSPRINGLS_H_
#define GLSHADERSPRINGLS_H_
#include "ImageSciUtil.h"
namespace imagesci {

class GLShaderSpringLS: public GLComponent {
private:
	GLShader mProgram;
	unsigned int FramebufferName;
	unsigned int depthrenderbuffer;
	unsigned int mTextureId;
	Camera* mCamera;
	Mesh* mMesh;
	std::vector<RGBA> mData;
public:
	GLShaderSpringLS(int x,int y,int w,int h,Camera* camera,Mesh* mesh);
	void updateGL();
	void render();
	virtual ~GLShaderSpringLS();
};

} /* namespace imagesci */

#endif /* GLSHADERSPRINGLS_H_ */
