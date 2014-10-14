///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2012-2013 DreamWorks Animation LLC
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of DreamWorks Animation nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////

#include "SimulationVisualizer.h"

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
void UpdateView(SimulationVisualizer* v){
	while(v->update()){
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
	setNeedsDisplay();
}

SimulationVisualizer::SimulationVisualizer()
    : mCamera(new Camera())
	, mMiniCamera(new Camera())
    , mShiftIsDown(false)
    , mCtrlIsDown(false)
    , mShowInfo(true)
	, mWin(NULL)
	, mUpdates(0)
{
}
void SimulationVisualizer::start(){
	mSimulation->reset();
	mMiniCamera->loadConfig();
	mSimulation->start();
	render();
}
void SimulationVisualizer::resume(){
	mMiniCamera->loadConfig();
	mSimulation->start();
	render();
}
SimulationVisualizer::~SimulationVisualizer(){
	if(mSimulation.get()!=nullptr)mSimulation->stop();
}
void SimulationVisualizer::stop(){
	mSimulation->stop();
}

bool SimulationVisualizer::init(int width,int height){
    if (glfwInit() != GL_TRUE) {
        std::cout<<"GLFW Initialization Failed.";
        return false;
    }


    mGridName.clear();
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

    glfwSetWindowTitle(mWin,mProgName.c_str());
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

    std::list<std::string> attrib;
	attrib.push_back("vp");
	attrib.push_back("vn");
	mIsoShader.Init("./matcap/JG_Red.png");

	mSpringlShader.Init("./matcap/JG_Silver.png");

    //Image* img=Image::read("buddha.png");
    //Text* txt=new Text(100,100,300,100);

	int mainW=1200;
	mPrettySpringlShader=std::unique_ptr<GLSpringlShader>(new GLSpringlShader(0,0,width,height));
	mPrettySpringlShader->setMesh(mCamera.get(),&mSimulation->getSource(),"./matcap/JG_Red.png","./matcap/JG_Silver.png");
	mPrettySpringlShader->updateGL();

	int miniW=256;
	int miniH=256;
	mIsoTexture=std::unique_ptr<GLFrameBuffer>(new GLFrameBuffer(width-miniW,0,miniW,miniH,miniW,miniH));
	std::vector<std::string> args;
	args.push_back("vp");
	args.push_back("uv");
	isoShader=std::unique_ptr<GLShader>(new GLShader());
	isoShader->Initialize(ReadTextFile("silhouette_shader.vert"),ReadTextFile("silhouette_shader.frag"),"",args);
	mIsoTexture->setShader(isoShader.get());
	mIsoTexture->updateGL();
	std::vector<RGBA> imgBuffer;
	int imgW,imgH;

	//mUI.init();

    //txt->setText("Hello World",14,true);

    glfwSetKeyCallback(mWin,keyCB);
    glfwSetMouseButtonCallback(mWin,mouseButtonCB);
    glfwSetCursorPosCallback(mWin,mousePosCB);
    glfwSetScrollCallback(mWin,mouseWheelCB);
    glfwSetWindowSizeCallback(mWin,windowSizeCB);
    glfwSetWindowRefreshCallback(mWin,windowRefreshCB);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glPointSize(4);
    glLineWidth(2);
	const float ambient[]={0.2f,0.2f,0.2f,1.0f};
	const float diffuse[]={0.8f,0.8f,0.8f,1.0f};
	const float specular[]={0.9f,0.9f,0.9f,1.0f};
	const float position[]={0.3f,0.5f,1.0f,0.0f};

	glEnable( GL_BLEND );

	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable( GL_MULTISAMPLE );

    size_t frame = 0;
    double time = glfwGetTime();
    glfwSwapInterval(1);
    do {
    	if(mSimulation->updateGL()){
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
SimulationVisualizer::setWindowTitle(double fps)
{
    std::ostringstream ss;
    ss  << mSimulation->getName()<<":: time="<<mSimulation->getSimulationTime()<<" iteration="<<mSimulation->getSimulationIteration();
    glfwSetWindowTitle(mWin,ss.str().c_str());
}


////////////////////////////////////////


void
SimulationVisualizer::render()
{


    const openvdb::BBoxd renderBBox=BBoxd(Vec3s(-0.5,-0.5,-0.5),Vec3s(0.5,0.5,0.5));

    openvdb::BBoxd bbox=mSimulation->getSource().mIsoSurface.GetBBox();
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

    bbox=mSimulation->getSource().mConstellation.GetBBox();
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
    const float specular=3;
    const float minZoom=0.25;
    const float maxZoom=2.0;
    float w=pow(cos(2*(t-0.5f)*M_PI)*0.5f+0.5f,specular);
    float zoom=w*(maxZoom-minZoom)+minZoom;
    if(mSimulation->isRunning())mCamera->setDistance(zoom);
    CameraAndSceneConfig p=mCamera->getConfig();
    p.mDistanceToObject=0.8f;
    p.mModelTranslation=Vec3d(0);
    p.mWorldTranslation=Vec3d(0);

    mMiniCamera->setConfig(p);
    mMiniCamera->setPose(miniPose.transpose());

    glEnable(GL_DEPTH_TEST);
	mIsoTexture->begin();
	mIsoShader.begin();
	mMiniCamera->aim(0,0,mIsoTexture->w,mIsoTexture->h,mIsoShader);
	mSimulation->getSource().mIsoSurface.draw();
	mIsoShader.end();
	mIsoTexture->end();

	/*
	mSpringlTexture->begin();
    mSpringlShader.begin();
	mMiniCamera->aim(0,0,mIsoTexture->w,mIsoTexture->h,mSpringlShader);
	springlGrid.draw(false,true,false,false);
	mSpringlShader.end();
	mSpringlTexture->end();
		*/
	mPrettySpringlShader->compute(mWin);

	glViewport(0,0,width,height);

	glEnable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	//bgImage->render(mWin);
	mPrettySpringlShader->render(mWin);
	//mSpringlTexture->render(mWin);
	mIsoTexture->render(mWin);

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
	/*
	mWireframeShader.begin();
	mCamera->aim(0,height/2,height/2,height/2,mWireframeShader);
	springlGrid.draw(false,true,false,false);
	mWireframeShader.end();
	*/
    //


	//mUI.aim(height/2,0,width-height/2,height);
    //mUI.render();



}


void
SimulationVisualizer::keyCallback(GLFWwindow* win,int key, int action,int mod)
{
    bool keyPress = (glfwGetKey(win,key) == GLFW_PRESS);
    mCamera->keyCallback(win,key, action);
    mShiftIsDown = glfwGetKey(win,GLFW_KEY_LEFT_SHIFT);
    mCtrlIsDown = glfwGetKey(win,GLFW_KEY_LEFT_CONTROL);

    if(keyPress){
		if(key==' '){
			if(mSimulation->isRunning()){
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
SimulationVisualizer::mouseButtonCallback(int button, int action)
{
    mCamera->mouseButtonCallback(button, action);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
SimulationVisualizer::mousePosCallback(int x, int y)
{
	mCamera->mousePosCallback(x, y);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
SimulationVisualizer::mouseWheelCallback(double pos)
{
        mCamera->mouseWheelCallback(pos);
        if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
SimulationVisualizer::windowSizeCallback(int, int)
{
    setNeedsDisplay();
}

////////////////////////////////////////


bool
SimulationVisualizer::needsDisplay()
{
    if (mUpdates < 2) {
        mUpdates += 1;
        return true;
    }
    return false;
}


void
SimulationVisualizer::setNeedsDisplay()
{
    mUpdates = 0;
}
}
