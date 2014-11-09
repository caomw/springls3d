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

#ifndef FLIPFLUIDSOLVER_H_
#define FLIPFLUIDSOLVER_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>
#include <openvdb/tools/DenseSparseTools.h>
#include "fluid_common.h"
#include "fluid_sorter.h"
#undef OPENVDB_REQUIRE_VERSION_NAME


namespace imagesci {
namespace fluid{
/*
 *
 */
class FlipFluidSolver {

	protected:
		double max_dens;
		const static float ALPHA ;
		const static float DT   ;
		const static float DENSITY;
		const static float GRAVITY ;
		MACGrid<float> mVol;
		MACGrid<float> mVolSave;
		RegularGrid<char> mA;
		RegularGrid<float> mL;
		RegularGrid<float> mPress;
		RegularGrid<openvdb::Vec4f> mWallNormal;
		int mGridSize;
		float WALL_THICK;
		std::unique_ptr<sorter> sort;
		std::vector<Object> objects;
		void placeObjects();
		void placeWalls();
		void damBreakTest();
		void computeDensity();
	public:
		std::vector<particlePtr> particles;
		FlipFluidSolver(int gridSize);
		void init();
		virtual ~FlipFluidSolver();
	};
}
} /* namespace imagesci */

#endif /* FLIPFLUIDSOLVER_H_ */
