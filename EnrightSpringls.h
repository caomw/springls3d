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

#ifndef SPRINGLS_VIEWER_VIEWER_HAS_BEEN_INCLUDED
#define SPRINGLS_VIEWER_VIEWER_HAS_BEEN_INCLUDED
#undef OPENVDB_REQUIRE_VERSION_NAME

#define GLFW_INCLUDE_GLU
#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetAdvect.h>
#include <openvdb/tools/LevelSetMeasure.h>
#include <openvdb/tools/LevelSetMorph.h>
#include <openvdb/tools/Morphology.h>
#include <openvdb/tools/PointAdvect.h>
#include <openvdb/tools/PointScatter.h>
#include <openvdb/tools/ValueTransformer.h>
#include <openvdb/tools/VectorTransformer.h>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include "LuxCamera.h"
#include "ClipBox.h"
#include "Font.h"
#include "Mesh.h"
#include "SpringLevelSet.h"
#include "SpringLevelSetAdvection.h"
#include "GLRenderUI.h"
#include "GLShaderSpringLS.h"
#include "GLEnvironmentalShader.h"
namespace imagesci{

typedef openvdb::tools::EnrightField<float> FieldT;
typedef SpringLevelSetAdvection<FieldT> AdvectT;

//template<typename Description> class SpringLevelSet;

class EnrightSpringls {
protected:
	static const float dt;
	float simTime;
	unsigned long simulationIteration;
	bool meshDirty;
	bool simulationRunning;
	bool playbackMode;
	std::mutex meshLock;
	int mUpdates;
	openvdb::BBoxd renderBBox;
	SpringLevelSet springlGrid;
	FieldT field;
	boost::shared_ptr<AdvectT> advect;
	std::thread simThread;
	std::string rootFile;
	std::unique_ptr<GLShaderSpringLS> mPrettySpringlShader;
	std::vector<std::string> isoSurfaceFiles;
	std::vector<std::string> constellationFiles;
	std::vector<std::string> signedDistanceFiles;
	openvdb::Mat4s Pose;
public:
	typedef std::unique_ptr<Camera> CameraPtr;

	typedef std::unique_ptr<openvdb_viewer::ClipBox> ClipBoxPtr;

	static EnrightSpringls* GetInstance();
	EnrightSpringls();
	~EnrightSpringls();
	bool update();
	void stash();
	bool openMesh(const std::string& fileName);
	bool openGrid(const std::string& fileName);
	bool openGrid(FloatGrid& fileName);
	bool openRecording(const std::string& dirName);
	bool needsDisplay();
	void setNeedsDisplay();
	void setWindowTitle(double fps = 0.0);
	void render();
	void resume();
	void setFrameIndex(int frameIdx);
	void updateCutPlanes(int wheelPos);
	bool init(int width, int height);
	void keyCallback(GLFWwindow* win,int key, int action,int mod);
	void mouseButtonCallback(int button, int action);
	void mousePosCallback(int x, int y);
	void mouseWheelCallback(double pos);
	void windowSizeCallback(int width, int height);
	void windowRefreshCallback();
	void start();
	void stop();
private:

    GLEnvironmentalShader mIsoShader;
    GLShader mSpringlShader;

    GLFWwindow* mWin;
	CameraPtr mCamera;
	GLRenderUI mUI;
	ClipBoxPtr mClipBox;
	std::string mGridName, mProgName, mGridInfo, mTransformInfo, mTreeInfo;
	bool mShiftIsDown, mCtrlIsDown, mShowInfo;
};
}
#endif
