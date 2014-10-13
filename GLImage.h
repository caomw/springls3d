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

#ifndef IMAGE_H_
#define IMAGE_H_
#include "ImageSciUtil.h"
#include "GLComponent.h"
#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include <openvdb/openvdb.h>
#include "GLShader.h"
namespace imagesci {
class GLImage:public GLComponent {
protected:
	std::vector<RGBA> mData;
	std::vector<RGBAf> mDataf;
	int mWidth;
	int mHeight;
	bool mShadeEnabled;
	static GLuint vao;
	static GLuint mPositionBuffer;
	static GLuint mUVBuffer;
	unsigned int mTextureId;
	bool mFloatType;
	GLShader* getShader();
public:
	static std::unique_ptr<GLShader> defaultShader;
	GLShader* imageShader;
	static const UV TextureCoords[6];
	static const openvdb::Vec3s PositionCoords[6];
	std::vector<RGBA>& data(){
		return mData;
	}
	inline void setShadeEnabled(bool shade){
		mShadeEnabled=shade;
	}
	void setShader(GLShader* shader){
		this->imageShader=shader;
	}
	inline int width(){return mWidth;}
	inline int height(){return mHeight;}
	unsigned int textureId(){return mTextureId;}
	RGBA& At(int i,int j){return mData[clamp(j,0,mHeight-1)*mWidth+clamp(i,0,mWidth-1)];}
	GLImage():GLComponent(),imageShader(NULL),mWidth(0),mHeight(0),mData(0),mTextureId(0),mFloatType(false),mShadeEnabled(true){

	}
	void setBounds(int _x,int _y,int _w,int _h){
		x=_x;
		y=_y;
		w=_w;
		h=_h;
	}
	GLImage(int _x,int _y,int _width,int _height,int imageWidth,int imageHeight,bool floatvType=false);
	GLImage(const std::vector<RGBA>& data,int width,int height);
	GLImage(const std::vector<RGBAf>& data,int width,int height);
	static GLImage* read(const std::string& file);
	bool write(const std::string& file);
	bool writeTexture(const std::string& file);
	virtual void updateGL();
	virtual void render(GLFWwindow* win);
	virtual ~GLImage();
};

} /* namespace imagesci */

#endif /* IMAGE_H_ */
