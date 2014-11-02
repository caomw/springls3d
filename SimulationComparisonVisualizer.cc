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
#include "SimulationComparisonVisualizer.h"
#include <openvdb/util/Formats.h> // for formattedInt()
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <openvdb/openvdb.h>

#include <tbb/mutex.h>
#include <iomanip> // for std::setprecision()
#include <iostream>
#include <sstream>
#include <vector>
#include <limits>
#include <boost/smart_ptr.hpp>
#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include <thread>

#include "GLImage.h"
#include "GLText.h"
namespace imagesci{

using namespace imagesci;
SimulationComparisonVisualizer* SimulationComparisonVisualizer::mSimVis=NULL;
SimulationComparisonVisualizer* SimulationComparisonVisualizer::getInstance(){
	if(mSimVis==NULL)mSimVis=new SimulationComparisonVisualizer();
	return mSimVis;
}
void ExecuteSimulationComparison(SimulationComparisonVisualizer* sim){
	try {
		sim->getSimulation1()->fireUpdateEvent();
		sim->getSimulation2()->fireUpdateEvent();
		bool ret1,ret2;
		sim->getLock().lock();
		ret1=sim->getSimulation1()->forceStep();
		ret2=sim->getSimulation2()->forceStep();
		sim->getLock().unlock();
		while(sim->isRunning()&&ret1&&ret2){
			sim->getSimulation1()->fireUpdateEvent();
			sim->getSimulation2()->fireUpdateEvent();

			sim->getLock().lock();
			ret1=sim->getSimulation1()->forceStep();
			ret2=sim->getSimulation2()->forceStep();
			sim->getLock().unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
	} catch (imagesci::Exception& e) {
		std::cout << "ImageSci Error:: "<< e.what() << std::endl;
	} catch (openvdb::Exception& e) {
		std::cout << "OpenVDB Error:: "<< e.what() << std::endl;
	}
}
void SimulationComparisonVisualizer::deleteInstance(){
	if(mSimVis!=NULL){
		delete mSimVis;
		mSimVis=NULL;
	}
}

void
keyCB2(GLFWwindow * win, int key, int scancode, int action, int mods)
{
	SimulationComparisonVisualizer::getInstance()->keyCallback(win,key, action,mods);
}


void
mouseButtonCB2(GLFWwindow* win,int button, int action,int mods)
{
	SimulationComparisonVisualizer::getInstance()->mouseButtonCallback(button, action);
}


void
mousePosCB2(GLFWwindow* win,double x, double y)
{
	SimulationComparisonVisualizer::getInstance()->mousePosCallback(x, y);
}


void
mouseWheelCB2(GLFWwindow* win,double x, double y)
{
	SimulationComparisonVisualizer::getInstance()->mouseWheelCallback(y);
}


void
windowSizeCB2(GLFWwindow* win,int width, int height)
{
	SimulationComparisonVisualizer::getInstance()->windowSizeCallback(width, height);
}


void
windowRefreshCB2(GLFWwindow* win)
{
	SimulationComparisonVisualizer::getInstance()->windowRefreshCallback();
}

using namespace openvdb;
using namespace openvdb::tools;

void SimulationComparisonVisualizer::windowRefreshCallback(){
	setNeedsDisplay();
}

SimulationComparisonVisualizer::SimulationComparisonVisualizer()
    : mCamera(new Camera())
	, mMiniCamera(new Camera())
	, mWin(NULL)
	, mSimulation1(NULL)
	, mSimulation2(NULL)
	, mUpdates(0)
	, mRunning(false)
{
}
void SimulationComparisonVisualizer::run(SimulationPlayback* simulation1,SimulationPlayback* simulation2,int width,int height){
	getInstance()->setSimulations(simulation1,simulation2);
	getInstance()->init(width,height);
	deleteInstance();
}
void SimulationComparisonVisualizer::start(){
	mSimulation1->reset();
	mSimulation2->reset();
    mSimulation1->init();
    mSimulation2->init();
	mMiniCamera->loadConfig();
	mRunning=true;
	mComparisonThread=std::thread(ExecuteSimulationComparison,this);
	render();
}
void SimulationComparisonVisualizer::resume(){
	if(mSimulation1!=NULL){
		mMiniCamera->loadConfig();
		mRunning=true;
		mComparisonThread=std::thread(ExecuteSimulationComparison,this);
	}
	render();
}
SimulationComparisonVisualizer::~SimulationComparisonVisualizer(){
	if(mSimulation1!=NULL)mSimulation1->stop();
	if(mSimulation2!=NULL)mSimulation2->stop();

}
void SimulationComparisonVisualizer::stop(){
	mRunning=false;
	if(mComparisonThread.joinable()){
		mComparisonThread.join();
	}
}
void SimulationComparisonVisualizer::SimulationEvent(Simulation* simulation,int mSimulationIteration,double time){

}
bool SimulationComparisonVisualizer::init(int width,int height){
    if (glfwInit() != GL_TRUE) {
        std::cout<<"GLFW Initialization Failed.";
        return false;
    }
    // Create window
    mWin=NULL;
    mRunning=false;
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_SAMPLES,8);
	glfwWindowHint(GLFW_DEPTH_BITS,32);
    if ((mWin=glfwCreateWindow(width, height,"Enright",NULL,NULL))==NULL)    // Window mode
    {
        glfwTerminate();
        return false;
    }
    GLint major,minor,rev;

    glfwSetWindowTitle(mWin,"Simulation Visualizer");
    glfwMakeContextCurrent(mWin);
    glfwSwapBuffers(mWin);

    std::cout<<"Draw "<<std::endl;
    mCamera->setSpeed(/*zoom=*/0.1, /*strafe=*/0.002, /*tumbling=*/0.2);
    mCamera->setNearFarPlanes(0.01f,1000.0f);
    mCamera->lookAt(Vec3d(0,0,0),1.0);
    mMiniCamera->setNearFarPlanes(0.1f,1000.0f);
    mMiniCamera->lookAt(Vec3d(0,0,0),1.0);
    mMiniCamera->setSpeed(/*zoom=*/0.1, /*strafe=*/0.002, /*tumbling=*/0.2);

    mMiniCamera->loadConfig();
    mCamera->loadConfig();

    std::list<std::string> attrib;
	attrib.push_back("vp");
	attrib.push_back("vn");
	mIsoShader.Init("./matcap/JG_Red.png");

	mSpringlShader.Init("./matcap/JG_Silver.png");
	int mainW=1200;

	mIsoTexture1=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(0,0,width/2,height,width/2,height));
	mIsoTexture2=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(width/2,0,width/2,height,width/2,height));

	std::vector<std::string> args;
	args.push_back("vp");
	args.push_back("uv");

	isoShader=std::unique_ptr<GLShader>(new GLShader());
	isoShader->Initialize(ReadTextFile("silhouette_shader.vert"),ReadTextFile("fade_shader.frag"),"",args);

	mIsoTexture1->setShader(isoShader.get());
	mIsoTexture1->updateGL();

	mIsoTexture2->setShader(isoShader.get());
	mIsoTexture2->updateGL();


	std::vector<RGBA> imgBuffer;
	int imgW,imgH;
    glfwSetKeyCallback(mWin,keyCB2);
    glfwSetMouseButtonCallback(mWin,mouseButtonCB2);
    glfwSetCursorPosCallback(mWin,mousePosCB2);
    glfwSetScrollCallback(mWin,mouseWheelCB2);
    glfwSetWindowSizeCallback(mWin,windowSizeCB2);
    glfwSetWindowRefreshCallback(mWin,windowRefreshCB2);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
	glEnable( GL_BLEND );
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable( GL_MULTISAMPLE );
    size_t frame = 0;
    double time = glfwGetTime();
    glfwSwapInterval(1);
    if(mSimulation1->getName()!="Recording")mSimulation1->addListener(this);

    start();
    do {

    	mLock.lock();
    	bool ret1=mSimulation1->updateGL();
    	bool ret2=mSimulation2->updateGL();
    	mLock.unlock();
    	if(ret1||ret2){
    		render();
        } else {
    		if(needsDisplay()){
    			render();
    		}
    	}
        ++frame;
        double elapsed = glfwGetTime() - time;
        if (elapsed > 1.0) {
            time = glfwGetTime();
            setWindowTitle(double(frame) / elapsed);
            frame = 0;
        }
        // Swap front and back buffers
        glfwSwapBuffers(mWin);
    // exit if the esc key is pressed or the window is closed.
        glfwPollEvents();
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (!glfwWindowShouldClose(mWin));
    mCamera->saveConfig();
    glfwTerminate();
    return true;
}

void
SimulationComparisonVisualizer::setWindowTitle(double fps)
{
    std::ostringstream ss;
    if(mSimulation1!=NULL){
    	ss  << mSimulation1->getName()<<"/"<<mSimulation2->getName()<<":: time="<<mSimulation1->getSimulationTime()<<" iteration="<<mSimulation1->getSimulationIteration();
    	glfwSetWindowTitle(mWin,ss.str().c_str());
    }
}


////////////////////////////////////////


void
SimulationComparisonVisualizer::render()
{

    const openvdb::BBoxd renderBBox=BBoxd(Vec3s(-0.5,-0.5,-0.5),Vec3s(0.5,0.5,0.5));
    if(mSimulation1->getSimulationIteration()==0){
    	mOriginalBoundingBox=mSimulation1->getSource().mIsoSurface.getBoundingBox();
    }
    openvdb::BBoxd bbox=mOriginalBoundingBox;
    openvdb::Vec3d extents = bbox.extents();
    openvdb::Vec3d rextents=renderBBox.extents();
    double scale = std::max(rextents[0], std::max(rextents[1], rextents[2]))/std::max(extents[0], std::max(extents[1], extents[2]));
    Vec3s minPt=bbox.getCenter();
    Vec3s rminPt=renderBBox.getCenter();
    Mat4s Pose;
    Pose.setIdentity();
    Pose.postTranslate(-minPt);
	Pose.postScale(Vec3s(scale,scale,scale));
	Pose.postTranslate(rminPt);
    extents = bbox.extents();
    scale = std::max(rextents[0], std::max(rextents[1], rextents[2]))/std::max(extents[0], std::max(extents[1], extents[2]));
    minPt=bbox.getCenter();
    openvdb::Mat4s miniPose;
    miniPose.setIdentity();
    miniPose.postTranslate(-minPt);
	miniPose.postScale(Vec3s(scale,scale,scale));
	miniPose.postTranslate(rminPt);

	int width,height;
	glfwGetWindowSize(mWin,&width, &height);
	glViewport(0,0,width,height);
	glClearColor(0.0,0.0,0.0,0.0);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	mSimulation1->getSource().mConstellation.setPose(Pose);
	mSimulation1->getSource().mIsoSurface.setPose(Pose);

	mSimulation2->getSource().mConstellation.setPose(Pose);
	mSimulation2->getSource().mIsoSurface.setPose(Pose);

    mCamera->setPose(Pose.transpose());
    float t=mSimulation1->getSimulationTime()/mSimulation1->getSimulationDuration();

    CameraAndSceneConfig p=mCamera->getConfig();
    p.mDistanceToObject=0.8f;
    p.mModelTranslation=Vec3d(0);
    p.mWorldTranslation=Vec3d(0);

    mMiniCamera->setConfig(p);

    mMiniCamera->setPose(miniPose.transpose());

    glEnable(GL_DEPTH_TEST);
	mIsoTexture1->begin();
	mIsoShader.begin();
	mMiniCamera->aim(0,0,mIsoTexture1->w,mIsoTexture1->h,mIsoShader);
	mSimulation1->getSource().mIsoSurface.draw();
	mIsoShader.end();
	mIsoTexture1->end();

	mIsoTexture2->begin();
	mIsoShader.begin();
	mMiniCamera->aim(0,0,mIsoTexture2->w,mIsoTexture2->h,mIsoShader);
	mSimulation2->getSource().mIsoSurface.draw();
	mIsoShader.end();
	mIsoTexture2->end();

	glViewport(0,0,width,height);
	glEnable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	mIsoTexture1->render(mWin);
	mIsoTexture2->render(mWin);

	/*
	if(mSimulation->isRunning()){
		std::stringstream ostr1,ostr2,ostr3;
		ostr1 <<  rootFile<<std::setw(4)<<std::setfill('0')<< (simulationIteration+1) << "_composite.png";
		std::vector<RGBA> tmp1(width*height);
		std::vector<RGBA> tmp2(width*height);
		glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, &tmp1[0]);
		for(int j=0;j<height;j++){
			for(int i=0;i<width;i++){
				RGBA c=tmp1[(height-1-j)*width+i];
				c[3]=255;
				tmp2[j*width+i]=c;
			}
		}
		WriteImageToFile(ostr1.str(),tmp2,width,height);
	}
	*/
}


void
SimulationComparisonVisualizer::keyCallback(GLFWwindow* win,int key, int action,int mod)
{
    bool keyPress = (glfwGetKey(win,key) == GLFW_PRESS);
    mCamera->keyCallback(win,key, action);
    if(keyPress){
		if(key==' '){
			if(mSimulation1->isRunning()){
				std::cout<<"############# STOP #############"<<std::endl;
				stop();
			} else {
				std::cout<<"############# START #############"<<std::endl;
				resume();
			}
		}
    }
    setNeedsDisplay();

}


void
SimulationComparisonVisualizer::mouseButtonCallback(int button, int action)
{
    mCamera->mouseButtonCallback(button, action);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
SimulationComparisonVisualizer::mousePosCallback(int x, int y)
{
	mCamera->mousePosCallback(x, y);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
SimulationComparisonVisualizer::mouseWheelCallback(double pos)
{
        mCamera->mouseWheelCallback(pos);
        if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
SimulationComparisonVisualizer::windowSizeCallback(int, int)
{
    setNeedsDisplay();
}

////////////////////////////////////////


bool
SimulationComparisonVisualizer::needsDisplay()
{
    if (mUpdates < 2) {
        mUpdates += 1;
        return true;
    }
    return false;
}


void
SimulationComparisonVisualizer::setNeedsDisplay()
{
    mUpdates = 0;
}
}
