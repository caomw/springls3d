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
#include "GLFrameBuffer.h"

namespace imagesci {

class GLShaderSpringLS: public GLComponent {
private:
	GLShader mNormalsAndDepthProgram;
	GLShader mWireframeProgram;
	GLShader mMixerProgram;

	std::unique_ptr<GLFrameBuffer> springlImage;
	std::unique_ptr<GLFrameBuffer> isoImage;
	std::unique_ptr<GLFrameBuffer> wireImage;
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
	void render(GLFWwindow* win);
	void compute(GLFWwindow* win);

	bool save(const std::string& file);
	virtual ~GLShaderSpringLS();
};

} /* namespace imagesci */

#endif /* GLSHADERSPRINGLS_H_ */
