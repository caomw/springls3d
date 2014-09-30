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
#include "Image.h"
namespace imagesci {

class GLShaderSpringLS: public GLComponent {
private:
	GLShader mNormalsAndDepthProgram;
	GLShader mWireframeProgram;
	GLShader mMixerProgram;
	unsigned int mFrameBufferId1;
	unsigned int mFrameBufferId2;
	unsigned int mFrameBufferId3;
	unsigned int mDepthBufferId1;
	unsigned int mDepthBufferId2;
	unsigned int mDepthBufferId3;

	std::unique_ptr<Image> springlImage;
	std::unique_ptr<Image> isoImage;
	std::unique_ptr<Image> wireImage;
	std::unique_ptr<Image> renderImage;

	unsigned int mMatCapId1;
	unsigned int mMatCapId2;
	Camera* mCamera;
	SpringLevelSet* mSpringLS;
	std::vector<openvdb::Vec4f> mData;
	std::string mSpringlMatcap;
	std::string mIsoMatcap;
public:
	GLShaderSpringLS(int x,int y,int w,int h);
	void setMesh(Camera* camera,SpringLevelSet* mesh,const std::string& springlMatcap,const std::string& isoMatcap);
	void updateGL();
	void render();
	bool save(const std::string& file);
	virtual ~GLShaderSpringLS();
};

} /* namespace imagesci */

#endif /* GLSHADERSPRINGLS_H_ */
