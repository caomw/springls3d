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

#include "Simulation.h"

namespace imagesci {
void ExecuteSimulation(Simulation* sim){
	sim->reset();
	while(sim->step()){
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

Simulation::Simulation():mRunning(false),mSimulationDuration(0),mSimulationTime(0),mSimulationIteration(0) {
	// TODO Auto-generated constructor stub

}
void Simulation::reset(){
	mSimulationTime=0;
	mSimulationIteration=0;
}
bool Simulation::stop(){
	if(!mRunning)return true;
	mRunning=false;
	if(mSimulationThread.joinable()){
		mSimulationThread.join();
	} else {
		return false;
	}
	return true;
}
bool Simulation::start(){
	if(!mRunning)return false;
	mRunning=true;
	mSimulationThread=std::thread(ExecuteSimulation,this);
	return true;
}
Simulation::~Simulation() {
	stop();
}

} /* namespace imagesci */
