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

#ifndef SIMULATIONCOMPARISONVISUALIZER_H_
#define SIMULATIONCOMPARISONVISUALIZER_H_
#include "Simulation.h"
#include "SimulationPlayback.h"
#include "graphics/Camera.h"
#include "graphics/GLEnvironmentalShader.h"
#include "graphics/GLFluidParticleShader.h"
#include "graphics/GLFrameBuffer.h"
#include "graphics/GLImage.h"
#include "graphics/GLText.h"
#include "graphics/GLSpringlShader.h"
#include "graphics/GLRenderUI.h"
#include "graphics/GLText.h"
#include <memory>
#include <string>
namespace imagesci {
class SimulationComparisonVisualizer: public SimulationListener {
private:

	int mUpdates;
    bool mShowParticles;
    bool mShowIsoSurface;
    bool mShowSpringls;
    GLEnvironmentalShader mIsoSurfaceShader;
    GLEnvironmentalShader mSpringlsShader;
    GLFluidParticleShader mParticleShader;
    GLShader mImageShader;

    std::unique_ptr<GLFrameBuffer> mIsoTexture1;
    std::unique_ptr<GLFrameBuffer> mIsoTexture2;
    std::unique_ptr<GLText> mSubtitle1;
    std::unique_ptr<GLText> mSubtitle2;
	//std::unique_ptr<GLShader> isoShader;
	std::unique_ptr<GLSpringlShader> mMultiPassShader1;
	std::unique_ptr<GLSpringlShader> mMultiPassShader2;
	std::unique_ptr<Camera> mCamera;
	std::unique_ptr<Camera> mMiniCamera;

    GLFWwindow* mWin;
	GLRenderUI mUI;
	openvdb::BBoxd mOriginalBoundingBox;
	SimulationPlayback* mSimulation1;
	SimulationPlayback* mSimulation2;
	static SimulationComparisonVisualizer* mSimVis;
	std::thread mComparisonThread;
	std::mutex mLock;
	bool mRunning;
	std::string mOutputDirectory;
	SimulationComparisonVisualizer();
public:
	std::mutex& getLock(){return mLock;}
	SimulationPlayback* getSimulation1(){
		return mSimulation1;
	}
	SimulationPlayback* getSimulation2(){
		return mSimulation2;
	}

	void SimulationEvent(Simulation* sim,int mSimulationIteration,double time);
	static SimulationComparisonVisualizer* getInstance();
	static void deleteInstance();
	static void run(SimulationPlayback* simulation1,SimulationPlayback* simulation2,int width,int height,const std::string& outputDir);
	void setSimulations(SimulationPlayback* simulation1,SimulationPlayback* simulation2){
		mSimulation1=simulation1;
		mSimulation2=simulation2;
	}
	void setOutputDirectory(const std::string& outputDir){
		mOutputDirectory=outputDir;
	}
	bool isRunning(){return mRunning;}
	bool needsDisplay();
	void setNeedsDisplay();
	void setWindowTitle(double fps = 0.0);
	void render();
	void resume();
	bool init(int width, int height);
	void keyCallback(GLFWwindow* win,int key, int action,int mod);
	void mouseButtonCallback(int button, int action);
	void mousePosCallback(int x, int y);
	void mouseWheelCallback(double pos);
	void windowSizeCallback(int width, int height);
	void windowRefreshCallback();
	void start();
	void stop();
	~SimulationComparisonVisualizer();
};

} /* namespace imagesci */

#endif /* SimulationComparisonVisualizer_H_ */
