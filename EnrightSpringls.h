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
#include "Mesh.h"
#include "SpringLevelSet.h"
#include "SpringLevelSetAdvection.h"
#include "GLRenderUI.h"
#include "GLSpringlShader.h"
#include "GLEnvironmentalShader.h"
#include "GLFrameBuffer.h"
#include "Camera.h"
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
	std::unique_ptr<GLImage> bgImage;
	std::unique_ptr<GLShader> isoShader;
	std::unique_ptr<GLShaderSpringLS> mPrettySpringlShader;
	std::vector<std::string> isoSurfaceFiles;
	std::vector<std::string> constellationFiles;
	std::vector<std::string> signedDistanceFiles;
	openvdb::Mat4s Pose;
public:
	typedef std::unique_ptr<Camera> CameraPtr;

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
    GLEnvironmentalShader mSpringlShader;
    std::unique_ptr<GLFrameBuffer> mIsoTexture;
    std::unique_ptr<GLFrameBuffer> mSpringlTexture;
    GLFWwindow* mWin;
	CameraPtr mCamera;
	CameraPtr mMiniCamera;
	GLRenderUI mUI;
	std::string mGridName, mProgName, mGridInfo, mTransformInfo, mTreeInfo;
	bool mShiftIsDown, mCtrlIsDown, mShowInfo;
};
}
#endif
