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
#undef OPENVDB_REQUIRE_VERSION_NAME

namespace openvdb {
	OPENVDB_USE_VERSION_NAMESPACE
	namespace OPENVDB_VERSION_NAME {
		namespace tools {
			template<typename ValueT, MemoryLayout Layout = LayoutZYX> class RegularGrid : public Dense<ValueT, Layout>{
				typedef DenseBase<ValueT, Layout> BaseT;
			private:
				ValueT* mPtr;
				size_t mStrideX;
				size_t mStrideY;
			public:
				RegularGrid(const Coord& dim, const Coord& min,ValueT value)
			        : Dense<ValueT, Layout>(dim,min)
			    {
					this->fill(value);
					mPtr=this->data();
					mStrideX=this->xStride();
					mStrideY=this->yStride();
			    }
				ValueT& operator()(size_t i,size_t j,size_t k){
					return mPtr[i*mStrideX + j*mStrideY + k];
				}
				const ValueT& operator()(size_t i,size_t j,size_t k) const{
					return mPtr[i*mStrideX + j*mStrideY + k];
				}
				ValueT& operator[](size_t i){
					return mPtr[i];
				}
				const ValueT& operator[](size_t i)const{
					return mPtr[i];
				}
			};
			template<typename ValueT, MemoryLayout Layout = LayoutZYX> struct MACGrid{
			public:
				RegularGrid<ValueT,Layout> mX,mY,mZ;
				MACGrid(const Coord& dim, const Coord& min,ValueT value)
			        : mX(dim,min,value),mY(dim,min,value),mZ(dim,min,value)
			    {
			    }
				RegularGrid<ValueT,Layout>& operator[](size_t i){
					return (&mX)[i];
				}

			};
		}
	}
}
namespace imagesci {

/*
 *
 */
class FlipFluidSolver {
protected:
	openvdb::tools::MACGrid<float> mDensity;
	openvdb::tools::MACGrid<float> mDensitySave;
	openvdb::tools::RegularGrid<char> mA;
	openvdb::tools::RegularGrid<float> mL;
	openvdb::tools::RegularGrid<float> mPress;
	openvdb::tools::RegularGrid<openvdb::Vec4f> mWallNormal;
	int mGridSize;
public:
	enum MaterialType {AIR=0,FLUID=1,WALL=2};
	FlipFluidSolver(int gridSize);
	void init();
	virtual ~FlipFluidSolver();
};

} /* namespace imagesci */

#endif /* FLIPFLUIDSOLVER_H_ */
