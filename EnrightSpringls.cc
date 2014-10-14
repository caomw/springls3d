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

#include "GLImage.h"
#include "GLText.h"
namespace imagesci{
EnrightSpringls* viewer=NULL;

const float EnrightSpringls::dt=0.005f;
using namespace imagesci;
EnrightSpringls* EnrightSpringls::GetInstance(){
	if(viewer==NULL)viewer=new EnrightSpringls();
	return viewer;
}
void UpdateView(EnrightSpringls* v){
	while(v->update()){
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
    : mCamera(new Camera())
	, mMiniCamera(new Camera())
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
	renderBBox=BBoxd(Vec3s(-0.5,-0.5,-0.5),Vec3s(0.5,0.5,0.5));
	Pose.setIdentity();
}
void EnrightSpringls::start(){
	advect=boost::shared_ptr<AdvectT>(new AdvectT(springlGrid,field));
	advect->setTemporalScheme(imagesci::TemporalIntegrationScheme::RK4b);

	simulationRunning=true;
	simThread=std::thread(UpdateView,this);
}
void EnrightSpringls::resume(){
	if(advect.get()==nullptr){
		advect=boost::shared_ptr<AdvectT>(new AdvectT(springlGrid,field));
		advect->setTemporalScheme(imagesci::TemporalIntegrationScheme::RK4b);
	}
	mMiniCamera->loadConfig();
	simulationRunning=true;
	render();
	simThread=std::thread(UpdateView,this);
}
EnrightSpringls::~EnrightSpringls(){
	stop();
}
void EnrightSpringls::stop(){
	if(!simulationRunning)return;
	simulationRunning=false;
	if(simThread.joinable()){
		simThread.join();
	}
}
bool EnrightSpringls::openMesh(const std::string& fileName){
	Mesh* mesh=new Mesh();
	mesh->openMesh(fileName);
	std::cout<<"Opened mesh "<<mesh->mVertexes.size()<<" "<<mesh->mFaces.size()<<" "<<mesh->mQuadIndexes.size()<<" "<<mesh->mTriIndexes.size()<<std::endl;
	if(mesh==NULL)return false;
	boost::shared_ptr<imagesci::Mesh> originalMesh=std::unique_ptr<Mesh>(mesh);
	originalMesh->mapIntoBoundingBox(originalMesh->estimateVoxelSize());
    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
    springlGrid.create(mesh);
    BBoxd bbox=springlGrid.mIsoSurface.updateBBox();
	trans=springlGrid.mSignedLevelSet->transformPtr();
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

bool EnrightSpringls::openGrid(FloatGrid& mSignedLevelSet){
    springlGrid.create(mSignedLevelSet);
    springlGrid.mIsoSurface.updateBBox();
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
	FloatGrid::Ptr mSignedLevelSet=boost::static_pointer_cast<FloatGrid>(ptr);
    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
    springlGrid.create(*mSignedLevelSet);
    springlGrid.mIsoSurface.updateBBox();
    //trans=springlGrid.mSignedLevelSet->transformPtr();
	springlGrid.transform()=springlGrid.mSignedLevelSet->transform();
	springlGrid.mSignedLevelSet->transform()=*trans;
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
   // printf("OpenGL version supported by this platform (%s): \n", glGetString(GL_VERSION));
    //BitmapFont13::initialize();
    openvdb::BBoxd bbox=renderBBox;
    openvdb::Vec3d extents = bbox.extents();
    double max_extent = std::max(extents[0], std::max(extents[1], extents[2]));

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
	mPrettySpringlShader->setMesh(mCamera.get(),&springlGrid,"./matcap/JG_Red.png","./matcap/JG_Silver.png");
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

    //stash();
    do {
       if(meshDirty){
    	   meshLock.lock();
    	   try {
				springlGrid.mIsoSurface.updateGL();
    	   } catch(Exception& e){
    		   std::cerr<<"Iso-Surface "<<e.what()<<std::endl;
    	   }
    	   try {
    		   springlGrid.mConstellation.updateGL();
    	   } catch(Exception& e){
    		   std::cerr<<"Constellation "<<e.what()<<std::endl;
    	   }
			meshLock.unlock();
			render();

			meshDirty=false;
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
bool EnrightSpringls::openRecording(const std::string& dirName){
	isoSurfaceFiles.clear();
	constellationFiles.clear();
	signedDistanceFiles.clear();
	int n1=GetDirectoryListing(dirName,isoSurfaceFiles,"_iso",".ply");
	int n2=GetDirectoryListing(dirName,constellationFiles,"_sls",".ply");
	int n3=GetDirectoryListing(dirName,signedDistanceFiles,"",".vdb");
	if(!(n1==n2&&n2==n3)||n1==0)return false;
	playbackMode=true;
	setFrameIndex(0);
	rootFile=GetFileWithoutExtension(signedDistanceFiles[0]);
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
	c.openMesh(constellationFiles[simulationIteration]);
	springlGrid.mConstellation.create(&c);
	springlGrid.mIsoSurface.openMesh(isoSurfaceFiles[simulationIteration]);
	springlGrid.mIsoSurface.updateVertexNormals(16);
	springlGrid.mConstellation.updateVertexNormals();

	openvdb::io::File file(signedDistanceFiles[simulationIteration]);
	file.open();
	openvdb::GridPtrVecPtr grids =file.getGrids();
	openvdb::GridPtrVec allGrids;
	allGrids.insert(allGrids.end(), grids->begin(), grids->end());
	GridBase::Ptr ptr = allGrids[0];
	FloatGrid::Ptr mSignedLevelSet=boost::static_pointer_cast<FloatGrid>(ptr);
	springlGrid.transform()=mSignedLevelSet->transform();
	mSignedLevelSet->setTransform(openvdb::math::Transform::createLinearTransform(1.0));
	springlGrid.mSignedLevelSet=mSignedLevelSet;
	springlGrid.mConstellation.updateBBox();
	meshLock.unlock();
	meshDirty=true;
	setNeedsDisplay();
}
bool EnrightSpringls::update(){
	if(!simulationRunning)return false;
	if(meshDirty){
		std::this_thread::sleep_for(std::chrono::milliseconds());
		return true;
	}
	std::cout<<"---------------------- Simulation Iteration ["<<simulationIteration<<"] ----------------------"<<std::endl;
	if(playbackMode){
		simTime=simulationIteration*dt;
		if(simulationIteration>=constellationFiles.size()){
			simulationIteration=0;
			simTime=0;
			simulationRunning=false;
		}
		//meshLock.lock();
		Mesh c;
		c.openMesh(constellationFiles[simulationIteration]);
		springlGrid.mConstellation.create(&c);
		springlGrid.mIsoSurface.openMesh(isoSurfaceFiles[simulationIteration]);
		springlGrid.mIsoSurface.updateVertexNormals(16);
		springlGrid.mConstellation.updateVertexNormals();
		meshDirty=true;
		while(meshDirty){
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		//meshLock.unlock();

		//setFrameIndex(simulationIteration);
		/*
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
		*/
	} else {
		if(!playbackMode)stash();
		advect->advect(simTime,simTime+dt);

	}
	springlGrid.mConstellation.updateBBox();
	simTime=dt*simulationIteration;
	meshDirty=true;
	setNeedsDisplay();
	simulationIteration++;
	if(simTime<=3.0f&&simulationRunning){
		return true;
	} else {
		stash();
		return false;
	}
}
void EnrightSpringls::stash(){

	std::ostringstream ostr1,ostr2,ostr3,ostr4,ostr5,ostr6,ostr7;
	ostr4 << rootFile <<std::setw(4)<<std::setfill('0')<< simulationIteration << ".lxs";
	//mCamera->setMaterialFile("/home/blake/materials/white_chess.lbm2");
	if(playbackMode){
		//mCamera->setGeometryFile(isoSurfaceFiles[simulationIteration],Pose);
	} else {
		ostr1 << rootFile<<"_sls" <<std::setw(4)<<std::setfill('0')<< simulationIteration << ".ply";
		springlGrid.mConstellation.save(ostr1.str());
		ostr2 <<  rootFile<<"_iso" <<std::setw(4)<<std::setfill('0')<< simulationIteration << ".ply";
		springlGrid.mIsoSurface.save(ostr2.str());


	//	ostr5<<  rootFile<<"_sgn" <<std::setw(4)<<std::setfill('0')<< simulationIteration;
		//WriteToRawFile(springlGrid.mSignedLevelSet,ostr5.str());

		//ostr6<<  rootFile<<"_usgn" <<std::setw(4)<<std::setfill('0')<< simulationIteration;
		//WriteToRawFile(springlGrid.mUnsignedLevelSet,ostr6.str());

		//ostr7<<  rootFile<<"_grad" <<std::setw(4)<<std::setfill('0')<< simulationIteration;
		//WriteToRawFile(springlGrid.gradient,ostr7.str());

		ostr3 <<  rootFile<<std::setw(4)<<std::setfill('0')<< simulationIteration << ".vdb";
		//mCamera->setGeometryFile(ostr2.str(),Pose);
		openvdb::io::File file(ostr3.str());
		openvdb::GridPtrVec grids;
		FloatGrid::Ptr mSignedLevelSet=boost::static_pointer_cast<FloatGrid>(springlGrid.mSignedLevelSet->copyGrid(CopyPolicy::CP_COPY));
		mSignedLevelSet->transform()=springlGrid.transform();
		//std::cout<<"Stash "<<springlGrid.mSignedLevelSet->transform()<<std::endl;
		grids.push_back(mSignedLevelSet);
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



    openvdb::BBoxd bbox=springlGrid.mIsoSurface.GetBBox();
    openvdb::Vec3d extents = bbox.extents();
    openvdb::Vec3d rextents=renderBBox.extents();
    double scale = std::max(rextents[0], std::max(rextents[1], rextents[2]))/std::max(extents[0], std::max(extents[1], extents[2]));
    Vec3s minPt=bbox.getCenter();
    Vec3s rminPt=renderBBox.getCenter();
    Pose.setIdentity();
    Pose.postTranslate(-minPt);
	Pose.postScale(Vec3s(scale,scale,scale));
	Pose.postTranslate(rminPt);

    bbox=springlGrid.mConstellation.GetBBox();
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

    springlGrid.mConstellation.setPose(Pose);
    springlGrid.mIsoSurface.setPose(Pose);

    mCamera->setPose(Pose.transpose());
    float t=simTime/3.0f;
    const float specular=3;
    const float minZoom=0.25;
    const float maxZoom=2.0;
    float w=pow(cos(2*(t-0.5f)*M_PI)*0.5f+0.5f,specular);
    float zoom=w*(maxZoom-minZoom)+minZoom;
    if(simulationRunning)mCamera->setDistance(zoom);
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
	springlGrid.mIsoSurface.draw();
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

	if(simulationRunning){
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





////////////////////////////////////////


void
EnrightSpringls::updateCutPlanes(int wheelPos)
{
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
		} else if(key==GLFW_KEY_HOME){
			if(playbackMode){
				mMiniCamera->loadConfig();
				setFrameIndex(0);
			}
		}  else if(key==GLFW_KEY_END){
			if(playbackMode){
				mMiniCamera->loadConfig();
				setFrameIndex(constellationFiles.size()-1);
			}
		}

    }

    setNeedsDisplay();

}


void
EnrightSpringls::mouseButtonCallback(int button, int action)
{
    mCamera->mouseButtonCallback(button, action);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
EnrightSpringls::mousePosCallback(int x, int y)
{
	mCamera->mousePosCallback(x, y);
    if (mCamera->needsDisplay()) setNeedsDisplay();
}


void
EnrightSpringls::mouseWheelCallback(double pos)
{
        mCamera->mouseWheelCallback(pos);
        if (mCamera->needsDisplay()) setNeedsDisplay();
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
