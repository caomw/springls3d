/*
 * GLRenderUI.h
 *
 *  Created on: Sep 22, 2014
 *      Author: blake
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
	void render();
	void init();
	void aim(GLFWwindow* win);
	void aim(int x,int y,int width,int height);
	virtual ~GLRenderUI();
};

} /* namespace imagesci */

#endif /* GLRENDERUI_H_ */
