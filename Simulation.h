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

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "SpringLevelSet.h"
#include <thread>
#include <mutex>
namespace imagesci {
class Simulation;
void ExecuteSimulation(Simulation* sim);
/*
 *
 */
class Simulation {

protected:
	std::string mName;
	SpringLevelSet mSource;
	double mSimulationTime;
	double mTimeStep;
	double mSimulationDuration;
	long mSimulationIteration;
	bool mRunning;
	bool mIsMeshDirty;
	std::thread mSimulationThread;

public:
	Simulation();
	void loadParameters(const std::string& paramFile);
	void saveParameters(const std::string& paramFile);
	bool setSource(const std::string& sourceFileName);
	inline bool isRunning(){return mRunning;}
	inline SpringLevelSet& getSource(){return mSource;}
	inline const std::string& getName(){return mName;}
	inline void setName(const std::string& name){mName=name;}
	inline double getSimulationTime(){return mSimulationTime;}
	inline double getSimulationDuration(){return mSimulationDuration;}
	inline long getSimulationIteration(){return mSimulationIteration;}
	virtual bool init()=0;
	virtual bool step()=0;
	bool updateGL();
	void reset();
	bool start();
	bool stop();
	virtual ~Simulation();
};

} /* namespace imagesci */

#endif /* SIMULATION_H_ */
