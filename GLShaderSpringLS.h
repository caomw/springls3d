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
#include "SpringLevelSet.h"
namespace imagesci {

class GLShaderSpringLS: public GLComponent {
private:
	GLShader mNormalsAndDepthProgram;
	GLShader mMixerProgram;
	unsigned int mFrameBufferId1;
	unsigned int mFrameBufferId2;
	unsigned int mDepthBufferId1;
	unsigned int mDepthBufferId2;
	unsigned int mIsoTextureId;
	unsigned int mSpringlTextureId;
	unsigned int mRenderTextureId;
	unsigned int mMatCapId1;
	unsigned int mMatCapId2;
	Camera* mCamera;
	SpringLevelSet* mSpringLS;
	std::vector<openvdb::Vec4f> mData;
public:
	GLShaderSpringLS(int x,int y,int w,int h);
	void setMesh(Camera* camera,SpringLevelSet* mesh);
	void updateGL();
	void render();
	virtual ~GLShaderSpringLS();
};

} /* namespace imagesci */

#endif /* GLSHADERSPRINGLS_H_ */
