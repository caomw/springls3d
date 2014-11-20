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
template<typename ValueT> class RegularGrid: public openvdb::tools::Dense<
		ValueT, openvdb::tools::MemoryLayout::LayoutZYX> {
private:
	ValueT* mPtr;
	const size_t mStrideX;
	const size_t mStrideY;
	const size_t mRows;
	const size_t mCols;
	const size_t mSlices;
	const float mVoxelSize;
	const openvdb::BBoxd mBoundingBox;
public:
	RegularGrid(const openvdb::Coord& dims, const openvdb::BBoxd& boundingBox,
			ValueT value=0.0) :
			openvdb::tools::Dense<ValueT,
					openvdb::tools::MemoryLayout::LayoutZYX>(dims, openvdb::Coord(0)),mBoundingBox(boundingBox) {
		this->fill(value);
		mPtr = this->data();
		mStrideX = this->xStride();
		mStrideY = this->yStride();
		mRows = dims[0];
		mCols = dims[1];
		mSlices = dims[2];
		//Assume isotropic voxels!
		mVoxelSize=(boundingBox.max()-boundingBox.min())[0]/dims[0];
	}
	RegularGrid(int rows, int cols, int slices,float voxelSize,ValueT value=0.0) :
			openvdb::tools::Dense<ValueT,
					openvdb::tools::MemoryLayout::LayoutZYX>(
					openvdb::Coord(rows, cols, slices), openvdb::Coord(0)),
					mStrideX(this->xStride()),
					mStrideY(this->yStride()),
					mRows(rows),
					mCols(cols),
					mSlices(slices),
					mVoxelSize(voxelSize),
					mBoundingBox(openvdb::Vec3d(0,0,0),openvdb::Vec3d(voxelSize*rows,voxelSize*cols,voxelSize*slices)) {
		mPtr = this->data();
		this->fill(value);
	}
	RegularGrid(const openvdb::Coord& dims,float voxelSize,ValueT value=0.0):RegularGrid(dims[0],dims[1],dims[2],voxelSize,value){

	}


	ValueT& operator()(size_t i, size_t j, size_t k) {
		return mPtr[i * mStrideX + j * mStrideY + k];
	}
	const ValueT& operator()(size_t i, size_t j, size_t k) const {
		return mPtr[i * mStrideX + j * mStrideY + k];
	}
	const openvdb::BBoxd& getBoundingBox() const {
		return mBoundingBox;
	}
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
	inline const float voxelSize() const {
		return mVoxelSize;
	}
	inline float interpolate(float x, float y, float z) {
		x = clamp(x,0.0f,(float)mRows);
		y = clamp(y,0.0f,(float)mCols);
		z = clamp(z,0.0f,(float)mSlices);
		int i = std::min((int)x,(int)mRows-1);
		int j = std::min((int)y,(int)mCols-1);
		int k = std::min((int)z,(int)mSlices-1);
		RegularGrid<ValueT>& q=*this;
		return	(k+1-z)*(((i+1-x)*q(i,j,k)+(x-i)*q(i+1,j,k))*(j+1-y) + ((i+1-x)*q(i,j+1,k)+(x-i)*q(i+1,j+1,k))*(y-j)) +
				(z-k)*(((i+1-x)*q(i,j,k+1)+(x-i)*q(i+1,j,k+1))*(j+1-y) + ((i+1-x)*q(i,j+1,k+1)+(x-i)*q(i+1,j+1,k+1))*(y-j));
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
protected:
	RegularGrid<ValueT> mX, mY, mZ;
	const size_t mRows;
	const size_t mCols;
	const size_t mSlices;
	const float mVoxelSize;
public:

	MACGrid(const openvdb::Coord& dim, float voxelSize,ValueT value=0.0) :
			mX(dim[0] + 1, dim[1], dim[2], voxelSize,value),
			mY(dim[0], dim[1] + 1, dim[2], voxelSize ,value),
			mZ(dim[0], dim[1], dim[2] + 1, voxelSize ,value),mRows(dim[0]),mCols(dim[1]),mSlices(dim[2]),mVoxelSize(voxelSize) {
	}
	RegularGrid<ValueT>& operator[](size_t i) {
		return (&mX)[i];
	}

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
	inline const float voxelSize() const {
		return mVoxelSize;
	}
	inline openvdb::Vec3s interpolate(const openvdb::Vec3s& p) {
		openvdb::Vec3s u;
		u[0] = mX.interpolate(mRows*p[0], mCols*p[1]-0.5, mSlices*p[2]-0.5);
		u[1] = mY.interpolate(mRows*p[0]-0.5, mCols*p[1], mSlices*p[2]-0.5);
		u[2] = mZ.interpolate(mRows*p[0]-0.5, mCols*p[1]-0.5, mSlices*p[2]);
		return u;
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
	openvdb::Vec3f mCenter;
	openvdb::Vec3f mBounds[2];
};

struct FluidParticle {
	openvdb::Vec3f mLocation;
	openvdb::Vec3f mVelocity;
	openvdb::Vec3f mNormal;
	char mObjectType;
	//char mVisible;
	bool mRemoveIndicator;
	//openvdb::Vec3f mTmp[2];
	float mMass;
	float mDensity;
};
typedef std::unique_ptr<FluidParticle> ParticlePtr;
}
}
#endif
