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

#include "EnrightSimulation.h"

namespace imagesci {

EnrightSimulation::EnrightSimulation() {
	// TODO Auto-generated constructor stub

}
bool EnrightSimulation::init(){
	int dim = 256;
	const float radius = 0.15f;
	const openvdb::Vec3f center(0.35f, 0.35f, 0.35f);
	float voxelSize = 1 / (float) (dim - 1);
	FloatGrid::Ptr mSignedLevelSet =openvdb::tools::createLevelSetSphere<FloatGrid>(radius,center, voxelSize);
    mSource=std::unique_ptr<SpringLevelSet>(new SpringLevelSet());
	mSource->create(*mSignedLevelSet);
    mSource->mIsoSurface.updateBBox();

	advect=std::unique_ptr<AdvectT>(new AdvectT(*mSource.get(),field));
	advect->setTemporalScheme(imagesci::TemporalIntegrationScheme::RK4b);
	mSimulationDuration=3.0f;
	mTimeStep=0.001;

	return true;
}
bool EnrightSimulation::step(){
	advect->advect(mSimulationTime,mSimulationTime+mTimeStep);
	mSimulationIteration++;
	mSimulationTime=mTimeStep*mSimulationIteration;
	if(mSimulationTime<=mSimulationDuration&&mRunning){
		return true;
	} else {
		return false;
	}
}

} /* namespace imagesci */
