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

#ifndef GLFLUIDPARTICLESHADER_H_
#define GLFLUIDPARTICLESHADER_H_
#include "GLShader.h"
namespace imagesci {

class GLFluidParticleShader: public GLShader{
protected:
	unsigned int mColormapId;
	float colorMapValue;
public:
	void setColorMapIndex(int i,bool flip){
		colorMapValue=(i+0.5f)/12.0f;
		if(flip)colorMapValue=-colorMapValue;
	}
	GLFluidParticleShader();
	bool Init(const std::string& colormapFile="./matcap/colormaps.png");
	virtual void begin();
	virtual void end();
	virtual ~GLFluidParticleShader();
};

} /* namespace imagesci */

#endif /* GLFLUIDPARTICLESHADER_H_ */
