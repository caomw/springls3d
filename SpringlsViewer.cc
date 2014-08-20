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

namespace imagesci {

const float SpringlsViewer::dt=0.005f;
void UpdateView(imagesci::SpringlsViewer* v){
	while(v->update()){
		std::this_thread::sleep_for(std::chrono::milliseconds(20 ));

	}
}


SpringlsViewer* sViewer = NULL;
tbb::mutex sLock;


void
keyCB(int key, int action)
{
	if (sViewer) sViewer->keyCallback(key, action);
}


void
mouseButtonCB(int button, int action)
{
	if (sViewer) sViewer->mouseButtonCallback(button, action);
}


void
mousePosCB(int x, int y)
{
	if (sViewer) sViewer->mousePosCallback(x, y);
}


void
mouseWheelCB(int pos)
{
	if (sViewer) sViewer->mouseWheelCallback(pos);
}


void
windowSizeCB(int width, int height)
{
	if (sViewer) sViewer->windowSizeCallback(width, height);
}


void
windowRefreshCB()
{
	if (sViewer) sViewer->windowRefreshCallback();
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
{
}
void SpringlsViewer::start(){
	simulationRunning=true;
	simTime=0.0f;
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
    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform(mesh->EstimateVoxelSize());
    std::cout<<"Voxel size "<<trans->voxelSize()<<std::endl;
    springlGrid.create(*originalMesh,trans);
    mClipBox->set(*springlGrid.signedLevelSet);
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
	/*
	advect=boost::shared_ptr<AdvectT>(new AdvectT(*springlGrid.signedLevelSet,field));
	advect->setSpatialScheme(openvdb::math::HJWENO5_BIAS);
	advect->setTemporalScheme(openvdb::math::TVD_RK2);
	advect->setTrackerSpatialScheme(openvdb::math::HJWENO5_BIAS);
	advect->setTrackerTemporalScheme(openvdb::math::TVD_RK1);
*/
   using namespace openvdb_viewer;

	meshDirty=false;
    mGridName.clear();
    // Create window
    if (!glfwOpenWindow(width, height,  // Window size
                       8, 8, 8, 8,      // # of R,G,B, & A bits
                       32, 0,           // # of depth & stencil buffer bits
                       GLFW_WINDOW))    // Window mode
    {
        glfwTerminate();
        std::cout<<"Could not Open "<<width<<" "<<height<<std::endl;
        return false;
    }

    glfwSetWindowTitle(mProgName.c_str());
    glfwSwapBuffers();

    BitmapFont13::initialize();

    openvdb::BBoxd bbox=mClipBox->GetBBox();
    std::cout<<"Bounding Box "<<bbox<<std::endl;
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
    glShadeModel(GL_SMOOTH);
    glPointSize(4);
    glLineWidth(2);
    size_t frame = 0;
    double time = glfwGetTime();
    glfwSwapInterval(1);
    do {
        if(meshDirty&&originalMesh.get()==nullptr){
			meshLock.lock();
			originalMesh->updateGL();
			meshDirty=false;
			meshLock.unlock();
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
    } while (!glfwGetKey(GLFW_KEY_ESC) && glfwGetWindowParam(GLFW_OPENED));
    glfwTerminate();
    return true;
}
bool SpringlsViewer::update(){
	meshLock.lock();
	advect->advect(simTime,simTime+dt);
	meshDirty=true;
	meshLock.unlock();
	simTime+=dt;
	setNeedsDisplay();
	return (simTime<=3.0f&&simulationRunning);
}

void
SpringlsViewer::setWindowTitle(double fps)
{
    std::ostringstream ss;
    ss  << mProgName << ": "
        << (mGridName.empty() ? std::string("OpenVDB") : mGridName)
        << std::setprecision(1) << std::fixed << fps << " fps";
    glfwSetWindowTitle(ss.str().c_str());
}


////////////////////////////////////////


void
SpringlsViewer::render()
{
    mCamera->aim();
    using namespace openvdb_viewer;

    mClipBox->render();
    mClipBox->enableClipping();
    originalMesh->draw(false);

    mClipBox->disableClipping();

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


}
