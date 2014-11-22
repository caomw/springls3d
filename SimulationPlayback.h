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

#ifndef SIMULATIONPLAYBACK_H_
#define SIMULATIONPLAYBACK_H_
#include "Simulation.h"
namespace imagesci {

/*
 *
 */
class SimulationPlayback:public Simulation {
protected:
	std::vector<std::string> mIsoSurfaceFiles;
	std::vector<std::string> mConstellationFiles;
	std::vector<std::string> mSignedDistanceFiles;
	std::vector<std::string> mParticleVolumeFiles;
	std::vector<SimulationTimeStepDescription> mTimeSteps;
	std::string mDirectory;
	Mesh mTemporaryMesh;
public:
	SimulationPlayback(const std::string& directory);
	virtual bool init();
	virtual bool step();
	virtual bool isPlayback(){return true;}
	inline bool forceStep(){
		mRunning=true;
		return step();
	}
	virtual void cleanup();
	virtual ~SimulationPlayback();
};

} /* namespace imagesci */

#endif /* SIMULATIONPLAYBACK_H_ */
