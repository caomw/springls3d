/*
 * Copyright(C) 2014, Blake C. Lucas, Ph.D. (img.science@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef GLSHADERSPRINGLS_H_
#define GLSHADERSPRINGLS_H_

#include "GLShader.h"
#include "GLComponent.h"

#include "Camera.h"
#include "../SpringLevelSet.h"
#include "../ImageSciUtil.h"
#include "../Mesh.h"
#include "GLImage.h"
#include "GLFrameBuffer.h"

namespace imagesci {

class GLSpringlShader: public GLComponent {
private:
	GLShader mNormalsAndDepthProgram;
	GLShader mWireframeProgram;
	GLShader mMixerProgram;

	std::unique_ptr<GLFrameBuffer> springlImage;
	std::unique_ptr<GLFrameBuffer> isoImage;
	std::unique_ptr<GLFrameBuffer> wireImage;
	std::unique_ptr<GLImage> renderImage;

	unsigned int mMatCapId1;
	unsigned int mMatCapId2;
	Camera* mCamera;
	SpringLevelSet* mSpringLS;
	std::vector<openvdb::Vec4f> mData;
	std::string mSpringlMatcap;
	std::string mIsoMatcap;
public:
	GLSpringlShader(int x,int y,int w,int h);
	void setMesh(Camera* camera,SpringLevelSet* mesh,const std::string& springlMatcap,const std::string& isoMatcap);
	void updateGL();
	void render(GLFWwindow* win);
	void compute(GLFWwindow* win);

	bool save(const std::string& file);
	virtual ~GLSpringlShader();
};

} /* namespace imagesci */

#endif /* GLSHADERSPRINGLS_H_ */
