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

#ifndef ARMADILLOTWIST_H_
#define ARMADILLOTWIST_H_
#include <openvdb/openvdb.h>
#include "SpringLevelSetFieldDeformation.h"
#include "TwistField.h"
#include "Simulation.h"
namespace imagesci {

/*
 *
 */
class ArmadilloTwist : public Simulation{
	typedef TwistField<float> FieldT;
	typedef SpringLevelSetFieldDeformation<FieldT> AdvectT;
private:
	std::unique_ptr<FieldT> mField;
	std::unique_ptr<AdvectT> mAdvect;
	std::string mSourceFileName;
	double mCycles;
public:
	bool init();
	bool step();
	void cleanup();
	ArmadilloTwist(const std::string& fileName,double cycles=1.0f,MotionScheme motionScheme=EXPLICIT);
	virtual ~ArmadilloTwist();
};

} /* namespace imagesci */

#endif /* ARMADILLOTWIST_H_ */
