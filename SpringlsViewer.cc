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
#undef OPENVDB_REQUIRE_VERSION_NAME
#include "SpringlsViewer.h"
#include "ImageSciUtil.h"
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
#if defined(__APPLE__) || defined(MACOSX)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <GL/glfw.h>

#include <chrono>
#include <thread>


SpringlsViewer* viewer=NULL;
const float SpringlsViewer::dt=0.005f;
using namespace imagesci;
using namespace openvdb_viewer;
SpringlsViewer* SpringlsViewer::GetInstance(){
	if(viewer==NULL)viewer=new SpringlsViewer();
	return viewer;
}
void UpdateView(SpringlsViewer* v){
	while(v->update()){
		std::this_thread::yield();
	}
}


void
keyCB(int key, int action)
{
	if (viewer) viewer->keyCallback(key, action);
}


void
mouseButtonCB(int button, int action)
{
	if (viewer) viewer->mouseButtonCallback(button, action);
}


void
mousePosCB(int x, int y)
{
	if (viewer) viewer->mousePosCallback(x, y);
}


void
mouseWheelCB(int pos)
{
	if (viewer) viewer->mouseWheelCallback(pos);
}


void
windowSizeCB(int width, int height)
{
	if (viewer) viewer->windowSizeCallback(width, height);
}


void
windowRefreshCB()
{
	if (viewer) viewer->windowRefreshCallback();
}

using namespace openvdb;
using namespace openvdb::tools;

void SpringlsViewer::windowRefreshCallback(){
	setNeedsDisplay();
}
SpringlsViewer::SpringlsViewer()
    : mCamera(new openvdb_viewer::Camera)
    , mClipBox(new openvdb_viewer::ClipBox)
    , mWheelPos(0)
    , mShiftIsDown(false)
    , mCtrlIsDown(false)
    , mShowInfo(true)
	, meshDirty(false)
	, simTime(0.0f)
	, mUpdates(1)
	, simulationRunning(false)
{
}
void SpringlsViewer::start(){
	simTime=0.0f;

	advect=boost::shared_ptr<AdvectT>(new AdvectT(*springlGrid.signedLevelSet,field));

	advect->setSpatialScheme(openvdb::math::HJWENO5_BIAS);
	advect->setTemporalScheme(openvdb::math::TVD_RK2);
	advect->setTrackerSpatialScheme(openvdb::math::HJWENO5_BIAS);
	advect->setTrackerTemporalScheme(openvdb::math::TVD_RK1);
	/*
	advect->setSpatialScheme(openvdb::math::FIRST_BIAS);
	advect->setTemporalScheme(openvdb::math::TVD_RK1);
	advect->setTrackerSpatialScheme(openvdb::math::FIRST_BIAS);
	advect->setTrackerTemporalScheme(openvdb::math::TVD_RK1);
	*/
	renderBBox=BBoxd(Vec3s(-50,-50,-50),Vec3s(50,50,50));
	simulationRunning=true;
	simThread=std::thread(UpdateView,this);

}
SpringlsViewer::~SpringlsViewer(){
	stop();
}
void SpringlsViewer::stop(){
	simulationRunning=false;
	if(simThread.joinable()){
		simThread.join();
	}
}
bool SpringlsViewer::openMesh(const std::string& fileName){
	Mesh* mesh=Mesh::openMesh(fileName);
	if(mesh==NULL)return false;
	originalMesh=std::unique_ptr<Mesh>(mesh);
	originalMesh->mapIntoBoundingBox(originalMesh->EstimateVoxelSize());
    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
    springlGrid.create(*originalMesh,trans);
    mClipBox->set(*springlGrid.signedLevelSet);
    imagesci::WriteToRawFile(springlGrid.signedLevelSet,"/home/blake/tmp/signedLevelSet");
    imagesci::WriteToRawFile(springlGrid.springlPointerGrid,"/home/blake/tmp/springlIndex");
	BBoxd bbox=mClipBox->GetBBox();
	trans=springlGrid.signedLevelSet->transformPtr();
    Vec3d extents=bbox.extents();
	double max_extent = std::max(extents[0], std::max(extents[1], extents[2]));

	double scale=1.0/max_extent;
	const float radius = 0.15f;
    const openvdb::Vec3f center(0.35f,0.35f,0.35f);

	Vec3s t=-0.5f*(bbox.min()+bbox.max());
	trans->postTranslate(t);
	trans->postScale(scale*2*radius);
	trans->postTranslate(center);
	bbox = worldSpaceBBox(springlGrid.signedLevelSet->transform(),springlGrid.signedLevelSet->evalActiveVoxelBoundingBox());
	mClipBox->setBBox(bbox);

	meshDirty=true;
    return true;
}
bool SpringlsViewer::openGrid(const std::string& fileName){
	Mesh* mesh=Mesh::openGrid(fileName);
	if(mesh==NULL)return false;
	originalMesh=std::unique_ptr<Mesh>(mesh);

    mClipBox->set(*springlGrid.signedLevelSet);

	return true;
}
bool SpringlsViewer::init(int width,int height){


    if (glfwInit() != GL_TRUE) {
        std::cout<<"GLFW Initialization Failed.";
        return false;
    }
    mGridName.clear();
    // Create window
    if (!glfwOpenWindow(width, height,  // Window size
                       8, 8, 8, 8,      // # of R,G,B, & A bits
                       32, 0,           // # of depth & stencil buffer bits
                       GLFW_WINDOW))    // Window mode
    {
        glfwTerminate();
        return false;
    }
    glfwSetWindowTitle(mProgName.c_str());
    glfwSwapBuffers();

    BitmapFont13::initialize();

    openvdb::BBoxd bbox=renderBBox;
    openvdb::Vec3d extents = bbox.extents();
    double max_extent = std::max(extents[0], std::max(extents[1], extents[2]));

    mCamera->setTarget(bbox.getCenter(), max_extent);
    mCamera->lookAtTarget();
    mCamera->setSpeed(/*zoom=*/0.1, /*strafe=*/0.002, /*tumbling=*/0.02);


    glfwSetKeyCallback(keyCB);
    glfwSetMouseButtonCallback(mouseButtonCB);
    glfwSetMousePosCallback(mousePosCB);
    glfwSetMouseWheelCallback(mouseWheelCB);
    glfwSetWindowSizeCallback(windowSizeCB);
    glfwSetWindowRefreshCallback(windowRefreshCB);
    glClearColor(0.85, 0.85, 0.85, 0.0f);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glPointSize(4);
    glLineWidth(2);
	const float ambient[]={0.2f,0.2f,0.2f,1.0f};
	const float diffuse[]={0.8f,0.8f,0.8f,1.0f};
	const float specular[]={0.9f,0.9f,0.9f,1.0f};
	const float position[]={0.3f,0.5f,1.0f,0.0f};
	glEnable(GL_POLYGON_SMOOTH);
	glEnable( GL_BLEND );
	glEnable(GL_NORMALIZE);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glEnable (GL_LINE_SMOOTH);
	glEnable( GL_COLOR_MATERIAL );
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_AMBIENT,(GLfloat*)&ambient);
	glLightfv(GL_LIGHT0, GL_SPECULAR,(GLfloat*)&specular);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,(GLfloat*)&diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION,(GLfloat*)&position);
	glMaterialf(GL_FRONT, GL_SHININESS, 5.0f);
    size_t frame = 0;
    double time = glfwGetTime();
    glfwSwapInterval(1);
    do {
       if(meshDirty&&originalMesh.get()!=nullptr){
			meshLock.lock();
				originalMesh->updateGL();
				springlGrid.constellation->updateGL();

			meshDirty=false;
			meshLock.unlock();
			render();
        } else {
    		if(needsDisplay())render();
    	}
        ++frame;
        double elapsed = glfwGetTime() - time;
        if (elapsed > 1.0) {
            time = glfwGetTime();
            setWindowTitle(double(frame) / elapsed);
            frame = 0;
        }
        // Swap front and back buffers
        glfwSwapBuffers();
    // exit if the esc key is pressed or the window is closed.
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (!glfwGetKey(GLFW_KEY_ESC) && glfwGetWindowParam(GLFW_OPENED));
    glfwTerminate();
    return true;
}
bool SpringlsViewer::update(){

	advect->advect(simTime,simTime+dt);
	simTime+=dt;
	meshLock.lock();
	springlGrid.constellation->create(springlGrid.signedLevelSet);
	meshDirty=true;
	meshLock.unlock();
	setNeedsDisplay();
	std::cout<<"Simulation Time "<<simTime<<std::endl;
	return (simTime<=3.0f&&simulationRunning);
}

void
SpringlsViewer::setWindowTitle(double fps)
{
    std::ostringstream ss;
    ss  << mProgName << ": "
        << (mGridName.empty() ? std::string("OpenVDB ") : mGridName)
        << std::setprecision(1) << std::fixed << fps << " fps";
    glfwSetWindowTitle(ss.str().c_str());
}


////////////////////////////////////////


void
SpringlsViewer::render()
{

    mCamera->aim();

    openvdb::BBoxd bbox=mClipBox->GetBBox();
    openvdb::Vec3d extents = bbox.extents();
    openvdb::Vec3d rextents=renderBBox.extents();

    double scale = std::max(rextents[0], std::max(rextents[1], rextents[2]))/std::max(extents[0], std::max(extents[1], extents[2]));
    Vec3s minPt=bbox.getCenter();
    Vec3s rminPt=renderBBox.getCenter();

    glPushMatrix();
    glTranslatef(rminPt[0],rminPt[1],rminPt[2]);
    glScalef(scale,scale,scale);
    glTranslatef(-minPt[0],-minPt[1],-minPt[2]);
    mClipBox->render();
    mClipBox->enableClipping();
	glColor3f(0.8f,0.8f,0.8f);
    //originalMesh->draw(false);
    springlGrid.draw(false);
    mClipBox->disableClipping();
    glPopMatrix();
    // Render text

        BitmapFont13::enableFontRendering();

        glColor3f (0.2, 0.2, 0.2);

        int width, height;
        glfwGetWindowSize(&width, &height);

        BitmapFont13::print(10, height - 13 - 10, mGridInfo);
        BitmapFont13::print(10, height - 13 - 30, mTransformInfo);
        BitmapFont13::print(10, height - 13 - 50, mTreeInfo);

        BitmapFont13::disableFontRendering();
}





////////////////////////////////////////


void
SpringlsViewer::updateCutPlanes(int wheelPos)
{
    double speed = std::abs(mWheelPos - wheelPos);
    if (mWheelPos < wheelPos) mClipBox->update(speed);
    else mClipBox->update(-speed);
    setNeedsDisplay();
}


////////////////////////////////////////


void
SpringlsViewer::keyCallback(int key, int action)
{
    OPENVDB_START_THREADSAFE_STATIC_WRITE

    mCamera->keyCallback(key, action);
    const bool keyPress = glfwGetKey(key) == GLFW_PRESS;
    mShiftIsDown = glfwGetKey(GLFW_KEY_LSHIFT);
    mCtrlIsDown = glfwGetKey(GLFW_KEY_LCTRL);

    if (keyPress) {
        switch (key) {
        case 'c': case 'C':
            mClipBox->reset();
            break;
        case 'h': case 'H': // center home
            mCamera->lookAt(openvdb::Vec3d(0.0), 10.0);
            break;
        case 'g': case 'G': // center geometry
            mCamera->lookAtTarget();
            break;
        }
    }

    switch (key) {
    case 'x': case 'X':
        mClipBox->activateXPlanes() = keyPress;
        break;
    case 'y': case 'Y':
        mClipBox->activateYPlanes() = keyPress;
        break;
    case 'z': case 'Z':
        mClipBox->activateZPlanes() = keyPress;
        break;
    }

    mClipBox->shiftIsDown() = mShiftIsDown;
    mClipBox->ctrlIsDown() = mCtrlIsDown;

    setNeedsDisplay();

    OPENVDB_FINISH_THREADSAFE_STATIC_WRITE
}


void
SpringlsViewer::mouseButtonCallback(int button, int action)
{
    mCamera->mouseButtonCallback(button, action);
    mClipBox->mouseButtonCallback(button, action);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
SpringlsViewer::mousePosCallback(int x, int y)
{
    bool handled = mClipBox->mousePosCallback(x, y);
    if (!handled) mCamera->mousePosCallback(x, y);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
SpringlsViewer::mouseWheelCallback(int pos)
{
    if (mClipBox->isActive()) {
        updateCutPlanes(pos);
    } else {
        mCamera->mouseWheelCallback(pos, mWheelPos);
        if (mCamera->needsDisplay()) setNeedsDisplay();
    }

    mWheelPos = pos;
}


void
SpringlsViewer::windowSizeCallback(int, int)
{
    setNeedsDisplay();
}

////////////////////////////////////////


bool
SpringlsViewer::needsDisplay()
{
    if (mUpdates < 2) {
        mUpdates += 1;
        return true;
    }
    return false;
}


void
SpringlsViewer::setNeedsDisplay()
{
    mUpdates = 0;
}

