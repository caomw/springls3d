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
 *
 *  This implementation of a PIC/FLIP fluid simulator is derived from:
 *
 *  Ando, R., Thurey, N., & Tsuruno, R. (2012). Preserving fluid sheets with adaptively sampled anisotropic particles.
 *  Visualization and Computer Graphics, IEEE Transactions on, 18(8), 1202-1214.
 */

#ifndef _FLUIDCOMMON_H
#define _FLUIDCOMMON_H
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>
#include "../ImageSciUtil.h"
#include <memory>
namespace imagesci {
namespace fluid {
template<typename ValueT> struct Offset1D {
private:
	ValueT* mPtr;
public:
	Offset1D(ValueT* ptr) :
			mPtr(ptr) {
	}
	inline ValueT& operator[](size_t k) {
		return mPtr[k];
	}
	inline const ValueT& operator[](size_t k) const {
		return mPtr[k];
	}
};
template<typename ValueT> struct Offset2D {
private:
	ValueT* mPtr;
	size_t strideY;
public:
	Offset2D(ValueT* ptr, int stride) :
			mPtr(ptr), strideY(stride) {
	}
	inline Offset1D<ValueT> operator[](size_t j) {
		return Offset1D<ValueT>(&mPtr[strideY * j]);
	}
	inline const Offset1D<ValueT> operator[](size_t j) const {
		return Offset1D<ValueT>(&mPtr[strideY * j]);
	}
};

template<typename ValueT> class RegularGrid: public openvdb::tools::Dense<
		ValueT, openvdb::tools::MemoryLayout::LayoutZYX> {
private:
	ValueT* mPtr;
	size_t mStrideX;
	size_t mStrideY;
	size_t mRows;
	size_t mCols;
	size_t mSlices;
public:
	RegularGrid(const openvdb::Coord& dim, const openvdb::Coord& min,
			ValueT value) :
			openvdb::tools::Dense<ValueT,
					openvdb::tools::MemoryLayout::LayoutZYX>(dim, min) {
		this->fill(value);
		mPtr = this->data();
		mStrideX = this->xStride();
		mStrideY = this->yStride();
		mRows = dim[0];
		mCols = dim[1];
		mSlices = dim[2];
	}
	RegularGrid(int rows, int cols, int slices) :
			openvdb::tools::Dense<ValueT,
					openvdb::tools::MemoryLayout::LayoutZYX>(
					openvdb::Coord(rows, cols, slices), openvdb::Coord(0)) {
		mPtr = this->data();
		mStrideX = this->xStride();
		mStrideY = this->yStride();
		mRows = rows;
		mCols = cols;
		mSlices = slices;
	}
	ValueT& operator()(size_t i, size_t j, size_t k) {
		return mPtr[i * mStrideX + j * mStrideY + k];
	}
	const ValueT& operator()(size_t i, size_t j, size_t k) const {
		return mPtr[i * mStrideX + j * mStrideY + k];
	}
	//Not a good idea.
	/*
	 inline Offset2D<ValueT> operator[](size_t i) {
	 return Offset2D<ValueT>(&mPtr[i * mStrideX], mStrideY);
	 }
	 inline const Offset2D<ValueT> operator[](size_t i) const {
	 return Offset2D<ValueT>(&mPtr[i * mStrideX], mStrideY);
	 }*/
	inline const size_t size() const {
		return mRows * mCols * mSlices;
	}
	inline const size_t rows() const {
		return mRows;
	}
	inline const size_t cols() const {
		return mCols;
	}
	inline const size_t slices() const {
		return mSlices;
	}
	void copyTo(RegularGrid<ValueT>& out) {
		ValueT* src = this->data();
		ValueT* dest = out.data();
		memcpy(dest, src, sizeof(ValueT) * size());
	}
	void add(RegularGrid<ValueT>& out) {
		ValueT* src = this->data();
		ValueT* dest = out.data();
		size_t N = size();
		OPENMP_FOR for (size_t n = 0; n < N; n++) {
			src[n] += dest[n];
		}
	}
	void subtract(RegularGrid<ValueT>& out) {
		ValueT* src = this->data();
		ValueT* dest = out.data();
		size_t N = size();
		OPENMP_FOR for (size_t n = 0; n < N; n++) {
			src[n] -= dest[n];
		}
	}
	void subtractFrom(RegularGrid<ValueT>& out) {
		ValueT* src = this->data();
		ValueT* dest = out.data();
		size_t N = size();
		OPENMP_FOR for (size_t n = 0; n < N; n++) {
			src[n] = dest[n] - src[n];
		}
	}
};
inline bool WriteToRawFile(RegularGrid<float>& dense,
		const std::string& fileName) {
	std::ostringstream vstr;
	vstr << fileName << ".raw";
	FILE* f = fopen(vstr.str().c_str(), "wb");
	openvdb::CoordBBox bbox = dense.bbox();
	std::cout << "Grid size " << dense.valueCount() << std::endl;
	openvdb::Coord dims = bbox.max() - bbox.min() + openvdb::Coord(1, 1, 1);
	std::cout << "Dimensions " << dims << std::endl;
	openvdb::Coord P(0, 0, 0);
	for (P[2] = bbox.min()[2]; P[2] <= bbox.max()[2]; ++P[2]) {
		for (P[1] = bbox.min()[1]; P[1] <= bbox.max()[1]; ++P[1]) {

			for (P[0] = bbox.min()[0]; P[0] <= bbox.max()[0]; ++P[0]) {
				float val = dense.getValue(P);
				fwrite(&val, sizeof(float), 1, f);
			}
		}
	}
	fclose(f);
	std::cout << vstr.str() << std::endl;
	std::stringstream sstr;
	sstr << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	sstr << "<!-- MIPAV header file -->\n";
	sstr
			<< "<image xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" nDimensions=\"3\">\n";
	sstr << "	<Dataset-attributes>\n";
	sstr << "		<Image-offset>0</Image-offset>\n";
	sstr << "		<Data-type>Float</Data-type>\n";
	sstr << "		<Endianess>Little</Endianess>\n";
	sstr << "		<Extents>" << dims[0] << "</Extents>\n";
	sstr << "		<Extents>" << dims[1] << "</Extents>\n";
	sstr << "		<Extents>" << dims[2] << "</Extents>\n";
	sstr << "		<Resolutions>\n";
	sstr << "			<Resolution>1.0</Resolution>\n";
	sstr << "			<Resolution>1.0</Resolution>\n";
	sstr << "			<Resolution>1.0</Resolution>\n";
	sstr << "		</Resolutions>\n";
	sstr << "		<Slice-spacing>1.0</Slice-spacing>\n";
	sstr << "		<Slice-thickness>0.0</Slice-thickness>\n";
	sstr << "		<Units>Millimeters</Units>\n";
	sstr << "		<Units>Millimeters</Units>\n";
	sstr << "		<Units>Millimeters</Units>\n";
	sstr << "		<Compression>none</Compression>\n";
	sstr << "		<Orientation>Unknown</Orientation>\n";
	sstr << "		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr << "		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr << "		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr << "		<Origin>0.0</Origin>\n";
	sstr << "		<Origin>0.0</Origin>\n";
	sstr << "		<Origin>0.0</Origin>\n";
	sstr << "		<Modality>Unknown Modality</Modality>\n";
	sstr << "	</Dataset-attributes>\n";
	sstr << "</image>\n";
	std::ofstream myfile;
	std::stringstream xmlFile;
	xmlFile << fileName << ".xml";
	myfile.open(xmlFile.str().c_str(), std::ios_base::out);
	myfile << sstr.str();
	myfile.close();
	std::cout << xmlFile.str() << std::endl;
	return true;
}
template<typename ValueT> struct MACGrid {
public:
	RegularGrid<ValueT> mX, mY, mZ;
	MACGrid(const openvdb::Coord& dim, const openvdb::Coord& min, ValueT value) :
			mX(openvdb::Coord(dim[0] + 1, dim[1], dim[2]), min, value), mY(
					openvdb::Coord(dim[0], dim[1] + 1, dim[2]), min, value), mZ(
					openvdb::Coord(dim[0], dim[1], dim[2] + 1), min, value) {
	}
	RegularGrid<ValueT>& operator[](size_t i) {
		return (&mX)[i];
	}

};
inline bool WriteToRawFile(MACGrid<float>& mac, const std::string& fileName) {
	bool r1 = WriteToRawFile(mac[0], fileName + "_x");
	bool r2 = WriteToRawFile(mac[1], fileName + "_y");
	bool r3 = WriteToRawFile(mac[2], fileName + "_z");
	return (r1 && r2 && r3);
}

enum ObjectType {
	AIR = 0, FLUID = 1, WALL = 2
};
enum MaterialType {
	GLASS = 0, GRAY = 1, RED = 2
};
enum ObjectShape {
	BOX = 0, SPHERE = 1
};
struct CollisionObject {
	ObjectType type;
	ObjectShape shape;
	MaterialType material;
	bool mVisible;
	float mRadius;
	openvdb::Vec3f mColor;
	openvdb::Vec3f mBounds[2];
};

struct FluidParticle {
	openvdb::Vec3f mLocation;
	openvdb::Vec3f mVelocity;
	openvdb::Vec3f mNormal;
	char mObjectType;
	char mVisible;
	char mRemoveIndicator;
	char mThinParticle;
	openvdb::Vec3f mTmp[2];
	float mMass;
	float mDensity;
};
typedef std::unique_ptr<FluidParticle> ParticlePtr;
}
}
#endif
