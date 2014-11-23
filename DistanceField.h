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

#ifndef DISTANCEFIELD_H_
#define DISTANCEFIELD_H_
#include "ImageSciUtil.h"
#include "BinaryMinHeap.h"
#include <openvdb/openvdb.h>
#include <list>
namespace imagesci {
	typedef Indexable<float> VoxelIndex;
	class DistanceField {
	private:
		enum Flags{ ALIVE = 1,NBAND = 2,FARAWAY = 3};
		BinaryMinHeap<float> heap;
		double march(double Nv, double Sv, double Ev, double Wv,double Fv, double Bv, int Nl, int Sl, int El, int Wl, int Fl, int Bl);
	public:
		DistanceField(openvdb::Coord dims):heap(dims[0],dims[1],dims[2]){}
		void solve(const RegularGrid<float>& vol,RegularGrid<float>& out, double maxDistance=openvdb::LEVEL_SET_HALF_WIDTH);
		std::unique_ptr<RegularGrid<float> > solve(const RegularGrid<float>& vol, double maxDistance=openvdb::LEVEL_SET_HALF_WIDTH){
			RegularGrid<float>* distField=new RegularGrid<float>(vol.dimensions(),vol.voxelSize(),0.0);
			return std::unique_ptr<RegularGrid<float> >(distField);
		}
	};
} /* namespace imagesci */

#endif /* DISTANCEFIELD_H_ */
