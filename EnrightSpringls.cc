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

#include "EnrightSpringls.h"

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

#include "Image.h"
#include "Text.h"
namespace imagesci{
EnrightSpringls* viewer=NULL;

const float EnrightSpringls::dt=0.005f;
using namespace imagesci;
using namespace openvdb_viewer;
EnrightSpringls* EnrightSpringls::GetInstance(){
	if(viewer==NULL)viewer=new EnrightSpringls();
	return viewer;
}
void UpdateView(EnrightSpringls* v){
	while(v->update()){
		std::this_thread::yield();
		//std::this_thread::sleep_for(std::chrono::milliseconds());
	}
}


void
keyCB(GLFWwindow * win, int key, int scancode, int action, int mods)
{
	if (viewer) viewer->keyCallback(win,key, action,mods);
}


void
mouseButtonCB(GLFWwindow* win,int button, int action,int mods)
{
	if (viewer) viewer->mouseButtonCallback(button, action);
}


void
mousePosCB(GLFWwindow* win,double x, double y)
{
	if (viewer) viewer->mousePosCallback(x, y);
}


void
mouseWheelCB(GLFWwindow* win,double x, double y)
{
	if (viewer) viewer->mouseWheelCallback(y);
}


void
windowSizeCB(GLFWwindow* win,int width, int height)
{
	if (viewer) viewer->windowSizeCallback(width, height);
}


void
windowRefreshCB(GLFWwindow* win)
{
	if (viewer) viewer->windowRefreshCallback();
}

using namespace openvdb;
using namespace openvdb::tools;

void EnrightSpringls::windowRefreshCallback(){
	setNeedsDisplay();
}

EnrightSpringls::EnrightSpringls()
    : mCamera(new LuxCamera())
    , mClipBox(new openvdb_viewer::ClipBox)
    , mShiftIsDown(false)
    , mCtrlIsDown(false)
    , mShowInfo(true)
	, meshDirty(false)
	, simTime(0.0f)
	, mUpdates(1)
	,simulationIteration(0)
	,playbackMode(false)
	, simulationRunning(false)
{
	renderBBox=BBoxd(Vec3s(-50,-50,-50),Vec3s(50,50,50));
	Pose.setIdentity();
}
void EnrightSpringls::start(){
	advect=boost::shared_ptr<AdvectT>(new AdvectT(springlGrid,field));
	advect->setTemporalScheme(SpringlTemporalIntegrationScheme::RK4b);
	simulationRunning=true;
	simThread=std::thread(UpdateView,this);
}
void EnrightSpringls::resume(){
	if(advect.get()==nullptr){
		advect=boost::shared_ptr<AdvectT>(new AdvectT(springlGrid,field));
		advect->setTemporalScheme(SpringlTemporalIntegrationScheme::RK4b);
	}
	simulationRunning=true;
	simThread=std::thread(UpdateView,this);
}
EnrightSpringls::~EnrightSpringls(){
	stop();
}
void EnrightSpringls::stop(){
	simulationRunning=false;
	if(simThread.joinable()){
		simThread.join();
	}
}
bool EnrightSpringls::openMesh(const std::string& fileName){
	Mesh* mesh=new Mesh();
	mesh->openMesh(fileName);
	std::cout<<"Opened mesh "<<mesh->vertexes.size()<<" "<<mesh->faces.size()<<" "<<mesh->quadIndexes.size()<<" "<<mesh->triIndexes.size()<<std::endl;
	if(mesh==NULL)return false;
	boost::shared_ptr<imagesci::Mesh> originalMesh=std::unique_ptr<Mesh>(mesh);
	originalMesh->mapIntoBoundingBox(originalMesh->EstimateVoxelSize());
    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
    springlGrid.create(mesh);
    mClipBox->set(*(springlGrid.signedLevelSet));
	BBoxd bbox=mClipBox->GetBBox();
	trans=springlGrid.signedLevelSet->transformPtr();
    Vec3d extents=bbox.extents();
	double max_extent = std::max(extents[0], std::max(extents[1], extents[2]));
	double scale=1.0/max_extent;
	const float radius = 0.15f;
    const openvdb::Vec3f center(0.35f,0.35f,0.35f);
	Vec3s t=-0.5f*(bbox.min()+bbox.max());
	trans=springlGrid.transformPtr();
	trans->postTranslate(t);
	trans->postScale(scale*2*radius);
	trans->postTranslate(center);
	meshDirty=true;
	setNeedsDisplay();
	rootFile=GetFileWithoutExtension(fileName);
    return true;
}

bool EnrightSpringls::openGrid(FloatGrid& signedLevelSet){
    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
    springlGrid.create(signedLevelSet);
    mClipBox->set(*(springlGrid.signedLevelSet));
	BBoxd bbox=mClipBox->GetBBox();
	rootFile="/home/blake/tmp/enright";
	meshDirty=true;
	setNeedsDisplay();
	return true;
}
bool EnrightSpringls::openGrid(const std::string& fileName){
	openvdb::io::File file(fileName);
	file.open();
	openvdb::GridPtrVecPtr grids = file.getGrids();
	openvdb::GridPtrVec allGrids;
	allGrids.insert(allGrids.end(), grids->begin(), grids->end());
	GridBase::Ptr ptr = allGrids[0];
	Mesh* mesh = new Mesh();
	FloatGrid::Ptr signedLevelSet=boost::static_pointer_cast<FloatGrid>(ptr);
    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
    springlGrid.create(*signedLevelSet);
    mClipBox->set(*(springlGrid.signedLevelSet));
	BBoxd bbox=mClipBox->GetBBox();
	//trans=springlGrid.signedLevelSet->transformPtr();
	springlGrid.transform()=springlGrid.signedLevelSet->transform();
	springlGrid.signedLevelSet->transform()=*trans;
	/*
    Vec3d extents=bbox.extents();
	double max_extent = std::max(extents[0], std::max(extents[1], extents[2]));
	double scale=1.0/max_extent;
	const float radius = 0.15f;
    const openvdb::Vec3f center(0.35f,0.35f,0.35f);
	Vec3s t=-0.5f*(bbox.min()+bbox.max());
	trans=springlGrid.transformPtr();
	trans->postTranslate(t);
	trans->postScale(scale*2*radius);
	trans->postTranslate(center);
	*/
	meshDirty=true;
	rootFile=GetFileWithoutExtension(fileName);
	setNeedsDisplay();
	return true;
}
bool EnrightSpringls::init(int width,int height){


	/*



*/
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
    if ((mWin=glfwCreateWindow(width, height,"Enright",NULL,NULL))==NULL)    // Window mode
    {
        glfwTerminate();
        return false;
    }
    GLint major,minor,rev;


    glfwSetWindowTitle(mWin,mProgName.c_str());
    glfwMakeContextCurrent(mWin);
    glfwSwapBuffers(mWin);
   // printf("OpenGL version supported by this platform (%s): \n", glGetString(GL_VERSION));
    //BitmapFont13::initialize();
    openvdb::BBoxd bbox=renderBBox;
    openvdb::Vec3d extents = bbox.extents();
    double max_extent = std::max(extents[0], std::max(extents[1], extents[2]));
    mCamera->setTarget(bbox.getCenter(), max_extent);
    mCamera->setNearFarPlanes(0.1f,500.0f);
    mCamera->lookAtTarget();
    mCamera->setSpeed(/*zoom=*/0.1, /*strafe=*/0.002, /*tumbling=*/0.02);
    mCamera->init();

    //Image* img=Image::read("buddha.png");
    //Text* txt=new Text(100,100,300,100);

/*
    try {
		mSpringlsShader=std::unique_ptr<GLShaderSpringLS>(new GLShaderSpringLS(height/2,0,width-height/2,height));
		mSpringlsShader->setMesh(mCamera.get(),&springlGrid);
		mSpringlsShader->updateGL();
 	 } catch(Exception& e){
		   std::cerr<<"Shader "<<e.what()<<std::endl;
	   }
*/
    //img->setBounds(0,0,100,100);
    //mUI.Add(img);
    //mUI.Add(txt);
    mUI.init();

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
	//glShadeModel(GL_SMOOTH);
	//glEnable( GL_COLOR_MATERIAL );

	/*

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	glLightfv(GL_LIGHT0, GL_AMBIENT,(GLfloat*)&ambient);
	glLightfv(GL_LIGHT0, GL_SPECULAR,(GLfloat*)&specular);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,(GLfloat*)&diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION,(GLfloat*)&position);
	glMaterialf(GL_FRONT, GL_SHININESS, 5.0f);
	*/
    size_t frame = 0;
    double time = glfwGetTime();
    glfwSwapInterval(1);
    if (GL_NO_ERROR != glGetError())
    			throw Exception("GL Error: AFTER UI INIT.");
    //stash();
    do {
       if(meshDirty){
    	   meshLock.lock();
    	   try {
    			if (GL_NO_ERROR != glGetError())
    					throw Exception("GL Error: BEFORE ISO UPDATE.");
				springlGrid.isoSurface.updateGL();
    	   } catch(Exception& e){
    		   std::cerr<<"Iso-Surface "<<e.what()<<std::endl;
      		   std::cerr<<"VERTS "<<springlGrid.isoSurface.vertexes.size()<<" QUADS "<<springlGrid.isoSurface.quadIndexes.size()<<std::endl;

    	   }
    	   try {
    		   springlGrid.constellation.updateGL();
    	   } catch(Exception& e){
    		   std::cerr<<"Constellation "<<e.what()<<std::endl;
    	   }
			meshDirty=false;
			meshLock.unlock();
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
    glfwTerminate();
    return true;
}
bool EnrightSpringls::openRecording(const std::string& dirName){
	isoSurfaceFiles.clear();
	constellationFiles.clear();
	signedDistanceFiles.clear();
	int n1=GetDirectoryListing(dirName,isoSurfaceFiles,"_iso",".ply");
	int n2=GetDirectoryListing(dirName,constellationFiles,"_sls",".ply");
	int n3=GetDirectoryListing(dirName,signedDistanceFiles,"",".vdb");
	std::cout<<"Files "<<n1<<" "<<n2<<" "<<n3<<std::endl;
	if(!(n1==n2&&n2==n3)||n1==0)return false;
	playbackMode=true;
	/*
	openGrid(signedDistanceFiles[0]);
	Mesh c;
	c.openMesh(constellationFiles[0]);
	springlGrid.constellation.create(&c);
	springlGrid.isoSurface.openMesh(isoSurfaceFiles[0]);
	springlGrid.isoSurface.updateVertexNormals();
	springlGrid.constellation.updateVertexNormals();
	meshDirty=true;
		simTime=0.0f;
		simulationIteration=0;
		advect=boost::shared_ptr<AdvectT>(new AdvectT(springlGrid,field));
		advect->setTemporalScheme(SpringlTemporalIntegrationScheme::RK4b);
	setNeedsDisplay();
	*/
	setFrameIndex(0);
	rootFile=GetFileWithoutExtension(signedDistanceFiles[0]);
	mClipBox->set(*(springlGrid.signedLevelSet));
	return true;
}
void EnrightSpringls::setFrameIndex(int frameIdx){
	simulationIteration=frameIdx;
	simTime=simulationIteration*dt;
	if(simulationIteration>=constellationFiles.size()){
		simulationIteration=0;
		simTime=0;
	}
	meshLock.lock();
	Mesh c;
	std::cout<<"Open Constellation "<<std::endl;
	c.openMesh(constellationFiles[simulationIteration]);
	springlGrid.constellation.create(&c);
	std::cout<<"Open Iso Surface "<<std::endl;
	springlGrid.isoSurface.openMesh(isoSurfaceFiles[simulationIteration]);

	std::cout<<"Update Constellation Normals"<<std::endl;
	springlGrid.constellation.updateVertexNormals();
	std::cout<<"Open VDB "<<std::endl;
	openvdb::io::File file(signedDistanceFiles[simulationIteration]);
	file.open();
	openvdb::GridPtrVecPtr grids =file.getGrids();
	openvdb::GridPtrVec allGrids;
	allGrids.insert(allGrids.end(), grids->begin(), grids->end());
	GridBase::Ptr ptr = allGrids[0];
	FloatGrid::Ptr signedLevelSet=boost::static_pointer_cast<FloatGrid>(ptr);
	springlGrid.transform()=signedLevelSet->transform();
	signedLevelSet->setTransform(openvdb::math::Transform::createLinearTransform(1.0));
	springlGrid.signedLevelSet=signedLevelSet;
	//WriteToRawFile(springlGrid.signedLevelSet,"/home/blake/signed_init");
	//WriteToRawFile(springlGrid.unsignedLevelSet,"/home/blake/unsigned_init");
	meshLock.unlock();
	std::cout<<"Update Surface"<<std::endl;
	meshDirty=true;
	setNeedsDisplay();
}
bool EnrightSpringls::update(){
	if(meshDirty){
		std::this_thread::sleep_for(std::chrono::milliseconds());
		return true;
	}
	std::cout<<"---------------------- Simulation Iteration ["<<simulationIteration<<"] ----------------------"<<std::endl;
	if(playbackMode){
		setFrameIndex(simulationIteration);
		std::ostringstream ostr1;
		int SZ=springlGrid.constellation.quadIndexes.size();
		springlGrid.constellation.uvMap.resize(SZ);
		for(int i=0;i<SZ;i+=4){
			springlGrid.constellation.uvMap[i]=Vec2s(0.2f,0.2f);
			springlGrid.constellation.uvMap[i+1]=Vec2s(0.8f,0.2f);
			springlGrid.constellation.uvMap[i+2]=Vec2s(0.8f,0.8f);
			springlGrid.constellation.uvMap[i+3]=Vec2s(0.2f,0.8f);

		}
		ostr1 << rootFile<<"_tex" <<std::setw(4)<<std::setfill('0')<< simulationIteration << ".ply";
		springlGrid.constellation.save(ostr1.str());
	} else {
		advect->advect(simTime,simTime+dt);
		stash();
	}
	simTime+=dt;
	meshDirty=true;
	setNeedsDisplay();
	simulationIteration++;
	return (simTime<=3.0f&&simulationRunning);
}
void EnrightSpringls::stash(){

	std::ostringstream ostr1,ostr2,ostr3,ostr4,ostr5,ostr6,ostr7;
	ostr4 << rootFile <<std::setw(4)<<std::setfill('0')<< simulationIteration << ".lxs";
	mCamera->setMaterialFile("/home/blake/materials/white_chess.lbm2");
	if(playbackMode){
		mCamera->setGeometryFile(isoSurfaceFiles[simulationIteration],Pose);
	} else {
		ostr1 << rootFile<<"_sls" <<std::setw(4)<<std::setfill('0')<< simulationIteration << ".ply";
		springlGrid.constellation.save(ostr1.str());
		ostr2 <<  rootFile<<"_iso" <<std::setw(4)<<std::setfill('0')<< simulationIteration << ".ply";
		springlGrid.isoSurface.save(ostr2.str());


	//	ostr5<<  rootFile<<"_sgn" <<std::setw(4)<<std::setfill('0')<< simulationIteration;
		//WriteToRawFile(springlGrid.signedLevelSet,ostr5.str());

		//ostr6<<  rootFile<<"_usgn" <<std::setw(4)<<std::setfill('0')<< simulationIteration;
		//WriteToRawFile(springlGrid.unsignedLevelSet,ostr6.str());

		//ostr7<<  rootFile<<"_grad" <<std::setw(4)<<std::setfill('0')<< simulationIteration;
		//WriteToRawFile(springlGrid.gradient,ostr7.str());

		ostr3 <<  rootFile<<std::setw(4)<<std::setfill('0')<< simulationIteration << ".vdb";
		mCamera->setGeometryFile(ostr2.str(),Pose);
		openvdb::io::File file(ostr3.str());
		openvdb::GridPtrVec grids;
		FloatGrid::Ptr signedLevelSet=boost::static_pointer_cast<FloatGrid>(springlGrid.signedLevelSet->copyGrid(CopyPolicy::CP_COPY));
		signedLevelSet->transform()=springlGrid.transform();
		//std::cout<<"Stash "<<springlGrid.signedLevelSet->transform()<<std::endl;
		grids.push_back(signedLevelSet);
		std::cout<<"Saving "<<ostr3.str()<<" ...";
		file.write(grids);
		std::cout<<"Done."<<std::endl;
	}
	//mCamera->write(ostr4.str(),640,640);
}
void
EnrightSpringls::setWindowTitle(double fps)
{
    std::ostringstream ss;
    ss  << "Enright - t="<<simTime<<" ["<<simulationIteration<<"]";
    glfwSetWindowTitle(mWin,ss.str().c_str());
}


////////////////////////////////////////


void
EnrightSpringls::render()
{



    openvdb::BBoxd bbox=mClipBox->GetBBox();
    openvdb::Vec3d extents = bbox.extents();
    openvdb::Vec3d rextents=renderBBox.extents();

    /*
    double scale = std::max(rextents[0], std::max(rextents[1], rextents[2]))/std::max(extents[0], std::max(extents[1], extents[2]));
    Vec3s minPt=bbox.getCenter();
    Vec3s rminPt=renderBBox.getCenter();
*/

    double scale = std::max(rextents[0], std::max(rextents[1], rextents[2]))/std::max(extents[0], std::max(extents[1], extents[2]));
    Vec3s minPt=bbox.getCenter();
    Vec3s rminPt=renderBBox.getCenter();

    Pose.setIdentity();
    Pose.postTranslate(-minPt);
	Pose.postScale(Vec3s(scale,scale,scale));
	Pose.postTranslate(rminPt);
	if (GL_NO_ERROR != glGetError())
			throw Exception("GL Error: BEFORE RENDER.");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    int width,height;

    glfwGetWindowSize(mWin,&width, &height);
    springlGrid.constellation.setPose(Pose);
    springlGrid.isoSurface.setPose(Pose);
    /*
try {
	mSpringlsShader->render();
 } catch(Exception& e){
	   std::cerr<<"Shader "<<e.what()<<std::endl;
  }
*/
    mCamera->aim(0,0,height/2,height/2);
    mCamera->setPose(Pose.transpose());
    mCamera->beginShader();
    /*
    getchar();

    for(openvdb::Vec3s vert:springlGrid.isoSurface.vertexes){
    	std::cout<<"Transformed "<<vert<<" "<<mCamera->transform(vert)<<std::endl;
    }
*/
	if (GL_NO_ERROR != glGetError())
			throw Exception("GL Error: AFTER AIM 1.");
	springlGrid.isoSurface.draw(false,false,false,false);
	mCamera->endShader();
	if (GL_NO_ERROR != glGetError())
			throw Exception("GL Error: AFTER DRAW 1.");
	 mCamera->beginShader();
    mCamera->aim(0,height/2,height/2,height/2);
	if (GL_NO_ERROR != glGetError())
			throw Exception("GL Error: AFTER AIM 2.");
	springlGrid.draw(false,true,false,false);
	 mCamera->endShader();
	if (GL_NO_ERROR != glGetError())
			throw Exception("GL Error: AFTER RENDER UPDATE.");
    //


	//mUI.aim(height/2,0,width-height/2,height);
    //mUI.render();

	//
    // Render text
    /*
        BitmapFont13::enableFontRendering();

        glColor3f (0.2, 0.2, 0.2);

        int width, height;
        glfwGetWindowSize(&width, &height);

        BitmapFont13::print(10, height - 13 - 10, mGridInfo);
        BitmapFont13::print(10, height - 13 - 30, mTransformInfo);
        BitmapFont13::print(10, height - 13 - 50, mTreeInfo);

        BitmapFont13::disableFontRendering();
        */
}





////////////////////////////////////////


void
EnrightSpringls::updateCutPlanes(int wheelPos)
{
    mClipBox->update(wheelPos);
    setNeedsDisplay();
}


////////////////////////////////////////


void
EnrightSpringls::keyCallback(GLFWwindow* win,int key, int action,int mod)
{
    bool keyPress = (glfwGetKey(win,key) == GLFW_PRESS);
    mCamera->keyCallback(win,key, action);
    mShiftIsDown = glfwGetKey(win,GLFW_KEY_LEFT_SHIFT);
    mCtrlIsDown = glfwGetKey(win,GLFW_KEY_LEFT_CONTROL);

    if(keyPress){
		if(key==' '){
			if(simulationRunning){
				std::cout<<"############# STOP #############"<<std::endl;
				stop();
			} else {
				std::cout<<"############# START #############"<<std::endl;
				resume();
			}
		} else if(key==GLFW_KEY_LEFT){
			setFrameIndex((simulationIteration-1+constellationFiles.size())%constellationFiles.size());
		}  else if(key==GLFW_KEY_RIGHT){
			setFrameIndex((simulationIteration+1)%constellationFiles.size());
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

}


void
EnrightSpringls::mouseButtonCallback(int button, int action)
{
    mCamera->mouseButtonCallback(button, action);
    mClipBox->mouseButtonCallback(button, action);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
EnrightSpringls::mousePosCallback(int x, int y)
{
    bool handled = mClipBox->mousePosCallback(x, y);
    if (!handled) mCamera->mousePosCallback(x, y);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
EnrightSpringls::mouseWheelCallback(double pos)
{

    if (mClipBox->isActive()) {
        updateCutPlanes(pos);
    } else {
        mCamera->mouseWheelCallback(pos);
        if (mCamera->needsDisplay()) setNeedsDisplay();
    }
}


void
EnrightSpringls::windowSizeCallback(int, int)
{
    setNeedsDisplay();
}

////////////////////////////////////////


bool
EnrightSpringls::needsDisplay()
{
    if (mUpdates < 2) {
        mUpdates += 1;
        return true;
    }
    return false;
}


void
EnrightSpringls::setNeedsDisplay()
{
    mUpdates = 0;
}
}
