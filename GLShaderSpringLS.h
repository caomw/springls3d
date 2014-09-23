/*
 * GLShaderSpringLS.h
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
 */

#ifndef GLSHADERSPRINGLS_H_
#define GLSHADERSPRINGLS_H_
#include "ImageSciUtil.h"
#include "GLShader.h"
#include "GLComponent.h"
#include "Mesh.h"
#include "Camera.h"
namespace imagesci {

class GLShaderSpringLS: public GLComponent {
private:
	GLShader mProgram;
	unsigned int mFrameBufferId;
	unsigned int mDepthBufferId;
	unsigned int mTextureId;
	Camera* mCamera;
	Mesh* mMesh;
	std::vector<openvdb::Vec4f> mData;
public:
	GLShaderSpringLS(int x,int y,int w,int h);
	void setMesh(Camera* camera,Mesh* mesh);
	void updateGL();
	void render();
	virtual ~GLShaderSpringLS();
};

} /* namespace imagesci */

#endif /* GLSHADERSPRINGLS_H_ */
