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
#ifndef GLRENDERUI_H_
#define GLRENDERUI_H_
#include "GLComponent.h"
#include <memory>
#include <vector>
#include <GLFW/glfw3.h>
namespace imagesci {

class GLRenderUI {
protected:
	int mScreenWidth, mScreenHeight;
	int x,y;
	std::vector<std::unique_ptr<GLComponent>> GLRenderComponents;
public:
	std::unique_ptr<GLComponent>& Add(GLComponent* comp){
		GLRenderComponents.push_back(std::unique_ptr<GLComponent>(comp));
		return *(GLRenderComponents.end());
	}
	std::vector<std::unique_ptr<GLComponent>>& GetComponents(){
		return GLRenderComponents;
	}
	GLRenderUI();
	void render(GLFWwindow* win);
	void init();
	void aim(GLFWwindow* win);
	void aim(int x,int y,int width,int height);
	virtual ~GLRenderUI();
};

} /* namespace imagesci */

#endif /* GLRENDERUI_H_ */
