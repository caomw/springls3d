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

#ifndef ENRIGHTSIMULATION_H_
#define ENRIGHTSIMULATION_H_
#include <openvdb/openvdb.h>
#include "SpringLevelSetAdvection.h"
#include "Simulation.h"
#include <memory>
namespace imagesci {
class EnrightSimulation:public Simulation {
	typedef openvdb::tools::EnrightField<float> FieldT;
	typedef SpringLevelSetAdvection<FieldT> AdvectT;
protected:
	FieldT mField;
	std::unique_ptr<AdvectT> mAdvect;
	int mGridSize;
	bool init();
	bool step();
	void cleanup();
public:
	EnrightSimulation(int gridSize=256,MotionScheme motionScheme=SEMI_IMPLICIT);
};

} /* namespace imagesci */

#endif /* ENRIGHTSIMULATION_H_ */
