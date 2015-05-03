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
#include "SimulationVisualizer.h"

#include <openvdb/util/Formats.h> // for formattedInt()
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <openvdb/openvdb.h>

#include <tbb/mutex.h>
#include <iomanip>
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

namespace imagesci{

using namespace imagesci;
SimulationVisualizer* SimulationVisualizer::mSimVis=NULL;
SimulationVisualizer* SimulationVisualizer::getInstance(){
	if(mSimVis==NULL)mSimVis=new SimulationVisualizer();
	return mSimVis;
}
void SimulationVisualizer::deleteInstance(){
	if(mSimVis!=NULL){
		delete mSimVis;
		mSimVis=NULL;
	}
}

void
keyCB(GLFWwindow * win, int key, int scancode, int action, int mods)
{
	SimulationVisualizer::getInstance()->keyCallback(win,key, action,mods);
}


void
mouseButtonCB(GLFWwindow* win,int button, int action,int mods)
{
	SimulationVisualizer::getInstance()->mouseButtonCallback(button, action);
}


void
mousePosCB(GLFWwindow* win,double x, double y)
{
	SimulationVisualizer::getInstance()->mousePosCallback(x, y);
}


void
mouseWheelCB(GLFWwindow* win,double x, double y)
{
	SimulationVisualizer::getInstance()->mouseWheelCallback(y);
}


void
windowSizeCB(GLFWwindow* win,int width, int height)
{
	SimulationVisualizer::getInstance()->windowSizeCallback(width, height);
}


void
windowRefreshCB(GLFWwindow* win)
{
	SimulationVisualizer::getInstance()->windowRefreshCallback();
}

using namespace openvdb;
using namespace openvdb::tools;

void SimulationVisualizer::windowRefreshCallback(){
	 mCamera->setNeedsDisplay(true);
}

SimulationVisualizer::SimulationVisualizer()
    : mCamera(new Camera())
	, mMiniCamera(new Camera())
	, mWin(NULL)
	, mSimulation(NULL)
	, mShowParticles(false)
	, mShowIsoSurface(true)
	, mShowSpringls(true)
	, mOutputDirectory("./")
{
}
void SimulationVisualizer::run(Simulation* simulation,int width,int height,const std::string outputDirectory){
	getInstance()->setSimulation(simulation);
	getInstance()->setOutputDirectory(outputDirectory);
	getInstance()->run(width,height);
	deleteInstance();
}
void SimulationVisualizer::start(){
	if(mSimulation!=NULL){
		mSimulation->reset();
		mMiniCamera->loadConfig();
		mSimulation->start();
	}
}
void SimulationVisualizer::resume(){
	if(mSimulation!=NULL){
		mMiniCamera->loadConfig();
		mSimulation->start();
	}
}
SimulationVisualizer::~SimulationVisualizer(){
	if(mSimulation!=NULL)mSimulation->stop();
}
void SimulationVisualizer::stop(){
	if(mSimulation!=NULL)mSimulation->stop();
}
void SimulationVisualizer::SimulationEvent(Simulation* simulation,int mSimulationIteration,double time){
	//std::cout<<"Stashing ..."<<std::endl;
	if(!mSimulation->isPlayback())simulation->stash(mOutputDirectory);
	while(mSimulation->isDirty()&&mSimulation->isRunning()){
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}
bool SimulationVisualizer::run(int width,int height){
    if (glfwInit() != GL_TRUE) {
        std::cout<<"GLFW Initialization Failed.";
        return false;
    }
    // Create window
    mWin=NULL;
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

    mCamera->setSpeed(/*zoom=*/0.1, /*strafe=*/0.002, /*tumbling=*/0.2);
    mCamera->setNearFarPlanes(0.01f,1000.0f);
    mCamera->lookAt(Vec3d(0,0,0),1.0);
    mMiniCamera->setNearFarPlanes(0.1f,1000.0f);
    mMiniCamera->lookAt(Vec3d(0,0,0),1.0);
    mMiniCamera->setSpeed(/*zoom=*/0.1, /*strafe=*/0.002, /*tumbling=*/0.2);

    mMiniCamera->loadConfig();
    mCamera->loadConfig();

 	mIsoSurfaceShader.Init("./matcap/JG_Silver.png");
	mSpringlsShader.Init("./matcap/JG_Silver.png");
	mParticleShader.Init();
	int colormap=8;
	mParticleShader.setColorMapIndex(colormap,true);
	mSpringlsShader.setColorMapIndex(colormap,true);
	std::vector<std::string> args;
	args.push_back("vp");
	args.push_back("uv");
	int mainW=1200;

 	mOverlayShader=std::unique_ptr<GLSpringlShader>(new GLSpringlShader(0,0,width,height));
	mOverlayShader->setMesh(mCamera.get(),&mSimulation->getSource(),"./matcap/JG_Silver.png","./matcap/JG_Silver.png");
	mOverlayShader->setColorMapIndex(colormap,true);
	mOverlayShader->updateGL();

	mImageShader.Initialize(ReadTextFile("shaders/image_shader.vert"),ReadTextFile("shaders/image_shader.frag"),"",args);
	mParticleTexture=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(0,0,width,height,width,height));
	mParticleTexture->setShader(&mImageShader);
	mParticleTexture->updateGL();

	int miniW=256;
	int miniH=256;
	mMiniViewTexture=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(width-miniW,0,miniW,miniH,miniW,miniH));

	//Set shader to use for background of texture
	mMiniViewShader.Initialize(ReadTextFile("shaders/silhouette_shader.vert"),ReadTextFile("shaders/silhouette_shader.frag"),"",args);
	mMiniViewTexture->setShader(&mMiniViewShader);
	mMiniViewTexture->updateGL();
	std::vector<RGBA> imgBuffer;
	int imgW,imgH;
    glfwSetKeyCallback(mWin,keyCB);
    glfwSetMouseButtonCallback(mWin,mouseButtonCB);
    glfwSetCursorPosCallback(mWin,mousePosCB);
    glfwSetScrollCallback(mWin,mouseWheelCB);
    glfwSetWindowSizeCallback(mWin,windowSizeCB);
    glfwSetWindowRefreshCallback(mWin,windowRefreshCB);
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
    if(mSimulation->getName()!="Recording")mSimulation->addListener(this);
    if(mSimulation->isPlayback()){
    	mSimulation->init();
    	mCamera->setNeedsDisplay(true);
    }else {
    	start();
    }
    //
    do {
    	if(mSimulation->updateGL()){
    		mSimulation->getSource().mConstellation.updateBoundingBox();
			render();
        } else {
    		if( mCamera->needsDisplay()){
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
SimulationVisualizer::setWindowTitle(double fps)
{
    std::ostringstream ss;
    if(mSimulation!=NULL){
    	ss  << mSimulation->getName()<<std::setprecision(6)<<" [SIMULATION TIME="<<mSimulation->getSimulationTime()<<"] [IITERATION="<<mSimulation->getSimulationIteration()<<"] [FRAME TIME="<<(mSimulation->getComputeTimePerFrame()*1000.0f)<<" msec]";
    	glfwSetWindowTitle(mWin,ss.str().c_str());
    }
}


////////////////////////////////////////


void
SimulationVisualizer::render()
{
	mDrawLock.lock();
	bool hasParticles=(mSimulation->getSource().mParticleVolume.mParticles.size()>0);
    const openvdb::BBoxd renderBBox=BBoxd(Vec3s(-0.5,-0.5,-0.5),Vec3s(0.5,0.5,0.5));
    if(mSimulation->getSimulationIteration()==0){
    	mOriginalBoundingBox=mSimulation->getSource().mIsoSurface.getBoundingBox();
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

	if(mSimulation->getSource().mConstellation.mVertexes.size()>0){
		bbox=mSimulation->getSource().mConstellation.getBoundingBox();
	} else {
		bbox=mSimulation->getSource().mIsoSurface.getBoundingBox();
	}

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

	mSimulation->getSource().mConstellation.setPose(Pose);
	mSimulation->getSource().mIsoSurface.setPose(Pose);

    mCamera->setPose(Pose.transpose());
    float t=mSimulation->getSimulationTime()/mSimulation->getSimulationDuration();

    CameraAndSceneConfig p=mCamera->getConfig();
    p.mDistanceToObject=0.8f;
    p.mModelTranslation=Vec3d(0);
    p.mWorldTranslation=Vec3d(0);
    mMiniCamera->setConfig(p);
    mMiniCamera->setPose(miniPose.transpose());
    glEnable(GL_DEPTH_TEST);

    if(hasParticles){
		if(mShowIsoSurface&&mShowSpringls){
			mOverlayShader->compute(mWin);
			glViewport(0,0,width,height);
			glDisable(GL_DEPTH_TEST);
			mOverlayShader->render(mWin);
		} else {
			mParticleTexture->begin();
			if(mShowParticles){
				mParticleShader.begin();
					mCamera->aim(0,0,mParticleTexture->w,mParticleTexture->h,mParticleShader);
					glUniform1f(glGetUniformLocation(mParticleShader.GetProgramHandle(),"minVelocity"),mSimulation->getSource().mParticleVolume.mMinVelocityMagnitude);
					glUniform1f(glGetUniformLocation(mParticleShader.GetProgramHandle(),"maxVelocity"),mSimulation->getSource().mParticleVolume.mMaxVelocityMagnitude);
					mSimulation->getSource().mParticleVolume.draw();
				mParticleShader.end();
			}
			if(mShowIsoSurface){
				mIsoSurfaceShader.begin();
					mCamera->aim(0,0,mParticleTexture->w,mParticleTexture->h,mIsoSurfaceShader);
					glUniform1f(glGetUniformLocation(mIsoSurfaceShader.GetProgramHandle(),"minVelocity"),0);
					glUniform1f(glGetUniformLocation(mIsoSurfaceShader.GetProgramHandle(),"maxVelocity"),0);
					glUniform1i(glGetUniformLocation(mIsoSurfaceShader.GetProgramHandle(),"transparent"),(mShowParticles)?1:0);
					mSimulation->getSource().mIsoSurface.draw();
				mIsoSurfaceShader.end();
			}
			if(mShowSpringls){
				mSpringlsShader.begin();
					mCamera->aim(0,0,mParticleTexture->w,mParticleTexture->h,mSpringlsShader);
					glUniform1i(glGetUniformLocation(mSpringlsShader.GetProgramHandle(),"transparent"),0);
					glUniform1f(glGetUniformLocation(mSpringlsShader.GetProgramHandle(),"minVelocity"),mSimulation->getSource().mParticleVolume.mMinVelocityMagnitude);
					glUniform1f(glGetUniformLocation(mSpringlsShader.GetProgramHandle(),"maxVelocity"),mSimulation->getSource().mParticleVolume.mMaxVelocityMagnitude);
					mSimulation->getSource().mConstellation.draw();
				mSpringlsShader.end();
			}
			mParticleTexture->end();
			glViewport(0,0,width,height);
			glDisable(GL_DEPTH_TEST);
			mParticleTexture->render(mWin);
		}
    } else {
		mMiniViewTexture->begin();
			mIsoSurfaceShader.begin();
				mMiniCamera->aim(0,0,mMiniViewTexture->w,mMiniViewTexture->h,mIsoSurfaceShader);
				mSimulation->getSource().mIsoSurface.draw();
			mIsoSurfaceShader.end();
		mMiniViewTexture->end();
		mOverlayShader->compute(mWin);
		glViewport(0,0,width,height);
		glDisable(GL_DEPTH_TEST);
		mOverlayShader->render(mWin);
		mMiniViewTexture->render(mWin);
    }
	/*
	if(isRunning()){
		std::stringstream ostr1,ostr2,ostr3;
		ostr1 << mOutputDirectory<<"sim_screenshot_"<<std::setw(8)<<std::setfill('0')<< mSimulation->getSimulationIteration() << ".png";
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
		if(WriteImageToFile(ostr1.str(),tmp2,width,height)){
			std::cout<<"Wrote "<<ostr1.str()<<std::endl;
		}
	}
	*/
	mCamera->setNeedsDisplay(false);
	mDrawLock.unlock();
}


void
SimulationVisualizer::keyCallback(GLFWwindow* win,int key, int action,int mod)
{
	mDrawLock.lock();
    bool keyPress = (glfwGetKey(win,key) == GLFW_PRESS);
    mCamera->keyCallback(win,key, action);
    if(keyPress){
		if(key==GLFW_KEY_ENTER){
			if(mSimulation->isRunning()){
				std::cout<<"############# STOP #############"<<std::endl;
				stop();
			} else {
				std::cout<<"############# START #############"<<std::endl;
				resume();
			}
		} else if(key=='P'){
			mShowParticles=!mShowParticles;
		}  else if(key=='I'){
			mShowIsoSurface=!mShowIsoSurface;
		}  else if(key=='E'){
			mShowSpringls=!mShowSpringls;
		} else if(key==GLFW_KEY_SPACE){
			mSimulation->step();
			mSimulation->fireUpdateEvent();
		}
    }
    mCamera->setNeedsDisplay(true);
    mDrawLock.unlock();

}


void
SimulationVisualizer::mouseButtonCallback(int button, int action)
{
	mDrawLock.lock();
    mCamera->mouseButtonCallback(button, action);
    mDrawLock.unlock();
}


void
SimulationVisualizer::mousePosCallback(int x, int y)
{
	mDrawLock.lock();
	mCamera->mousePosCallback(x, y);
	mDrawLock.unlock();
}


void
SimulationVisualizer::mouseWheelCallback(double pos)
{
	mDrawLock.lock();
	mCamera->mouseWheelCallback(pos);
    mDrawLock.unlock();
}


void
SimulationVisualizer::windowSizeCallback(int, int)
{
	mDrawLock.lock();
    mCamera->setNeedsDisplay(true);
    mDrawLock.unlock();
}

}
