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

#ifndef SIMULATIONVISUALIZER_H_
#define SIMULATIONVISUALIZER_H_
#include "Simulation.h"
#include "Camera.h"
#include "GLEnvironmentalShader.h"
#include "GLFrameBuffer.h"
#include "GLImage.h"
#include "GLSpringlShader.h"
#include "GLRenderUI.h"
#include <memory>
namespace imagesci {
class SimulationVisualizer {
private:

	int mUpdates;
    GLEnvironmentalShader mIsoShader;
    GLEnvironmentalShader mSpringlShader;
    std::unique_ptr<GLFrameBuffer> mIsoTexture;
    std::unique_ptr<GLFrameBuffer> mSpringlTexture;
	std::unique_ptr<GLImage> bgImage;
	std::unique_ptr<GLShader> isoShader;
	std::unique_ptr<GLSpringlShader> mPrettySpringlShader;
	std::unique_ptr<Camera> mCamera;
	std::unique_ptr<Camera> mMiniCamera;

    GLFWwindow* mWin;
	GLRenderUI mUI;
	std::string mGridName, mProgName, mGridInfo, mTransformInfo, mTreeInfo;
	bool mShiftIsDown, mCtrlIsDown, mShowInfo;
	std::unique_ptr<Simulation> mSimulation;
	static SimulationVisualizer* mSimVis;
	SimulationVisualizer();
public:
	static SimulationVisualizer* getInstance();
	static void deleteInstance();
	bool update();
	void stash();
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
	virtual ~SimulationVisualizer();
};

} /* namespace imagesci */

#endif /* SIMULATIONVISUALIZER_H_ */
