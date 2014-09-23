/*
 * GLComponent.h
 *
 *  Created on: Sep 21, 2014
 *      Author: blake
 */

#ifndef GLCOMPONENT_H_
#define GLCOMPONENT_H_
#include <vector>
#include <list>
#include <memory>
namespace imagesci {
//
class GLComponent {
protected:

public:
	int x;
	int y;
	int w;
	int h;

	virtual ~GLComponent();
	GLComponent() : x(0), y(0), w(0), h(0) {}
	GLComponent(int _x,int _y,int _w,int _h) : x(_x), y(_y), w(_w), h(_h) {}
	virtual void render()=0;
	virtual void updateGL()=0;
};

class GLComponentGroup: public GLComponent {
protected:
	std::list<std::unique_ptr<GLComponent>> components;
public:
	GLComponentGroup() :
			GLComponent() {

	}
	void render() {
		for (std::unique_ptr<GLComponent>& comp : components) {
			comp->render();
		}
	}
	void updateGL() {
		for (std::unique_ptr<GLComponent>& comp : components) {
			comp->updateGL();
		}
	}
	~GLComponentGroup(){}
};

} /* namespace imagesci */

#endif /* GLCOMPONENT_H_ */
