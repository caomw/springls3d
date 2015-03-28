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
#ifndef IMAGESCIUTIL_H_
#define IMAGESCIUTIL_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>

#ifdef MP
#include <omp.h>
#define OPENMP_FOR		_Pragma("omp parallel for" )
#define OPENMP_SECTION  _Pragma("omp section" )
#define OPENMP_BEGIN	_Pragma("omp parallel" ) {
#define OPENMP_END		}
#define OPENMP_FOR_P	_Pragma("omp for" )
#else
#define OPENMP_FOR
#define OPENMP_SECTION
#define OPENMP_BEGIN
#define OPENMP_END
#define OPENMP_FOR_P
#endif
#define FOR_EVERY_GRID_CELL(G)		for( int i=0; i<G.rows(); i++ ) for( int j=0; j<G.cols(); j++ ) for( int k=0; k<G.slices(); k++ ) {
#define END_FOR }
namespace imagesci {
typedef openvdb::math::Vec4<unsigned char> RGBA;
typedef openvdb::math::Vec4<float> RGBAf;
typedef openvdb::math::Vec2<float> UV;
typedef openvdb::math::Vec3<float> float3;
typedef openvdb::math::Vec4<float> float4;
typedef openvdb::math::Mat4f Matrix4f;
struct MakeString{
	std::ostringstream ss;
	operator std::string() const { return ss.str(); }
	template<class T> MakeString & operator << (const T & val) { ss << val; return *this; }
};
template<typename T> T clamp(T val, T min, T max) {
	return std::min(std::max(val, min), max);
}
template<typename T> openvdb::math::Vec2<T> clamp(openvdb::math::Vec2<T> val, T min, T max) {
	return openvdb::math::Vec2<T>(
			std::min(std::max(val[0], min), max),
			std::min(std::max(val[1], min), max));
}
template<typename T> openvdb::math::Vec3<T> clamp(openvdb::math::Vec3<T> val, T min, T max) {
	return openvdb::math::Vec3<T>(
			std::min(std::max(val[0], min), max),
			std::min(std::max(val[1], min), max),
			std::min(std::max(val[2], min), max));
}
template<typename T> openvdb::math::Vec4<T> clamp(openvdb::math::Vec4<T> val, T min, T max) {
	return openvdb::math::Vec4<T>(
			std::min(std::max(val[0], min), max),
			std::min(std::max(val[1], min), max),
			std::min(std::max(val[2], min), max),
			std::min(std::max(val[3], min), max));
}
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
	openvdb::math::Transform::Ptr mTransform;
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
		mTransform=openvdb::math::Transform::createLinearTransform(mVoxelSize);
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
		mTransform=openvdb::math::Transform::createLinearTransform(mVoxelSize);
		this->fill(value);
	}
	RegularGrid(const openvdb::Coord& dims,float voxelSize,ValueT value=0.0):RegularGrid(dims[0],dims[1],dims[2],voxelSize,value){

	}

	inline openvdb::math::Transform& transform() {
		return *mTransform;
	}
	inline openvdb::math::Transform::Ptr transformPtr() {
		return mTransform;
	}
	inline void setTrasnfrom(openvdb::math::Transform::Ptr transform){
		mTransform=transform;
	}
	ValueT& operator()(size_t i, size_t j, size_t k) {
		assert((i>=0&&i<mRows));
		assert((j>=0&&j<mCols));
		assert((k>=0&&k<mSlices));
		return mPtr[i * mStrideX + j * mStrideY + k];
	}
	const ValueT& operator()(size_t i, size_t j, size_t k) const {
		assert((i>=0&&i<mRows));
		assert((j>=0&&j<mCols));
		assert((k>=0&&k<mSlices));
		return mPtr[i * mStrideX + j * mStrideY + k];
	}
	ValueT& operator()(const openvdb::Coord& ijk) {
		assert((ijk[0]>=0&&ijk[0]<mRows));
		assert((ijk[1]>=0&&ijk[1]<mCols));
		assert((ijk[2]>=0&&ijk[2]<mSlices));
		return mPtr[ijk[0] * mStrideX + ijk[1] * mStrideY + ijk[2]];
	}
	const ValueT& operator()(const openvdb::Coord& ijk) const {
		assert((ijk[0]>=0&&ijk[0]<mRows));
		assert((ijk[1]>=0&&ijk[1]<mCols));
		assert((ijk[2]>=0&&ijk[2]<mSlices));
		return mPtr[ijk[0] * mStrideX + ijk[1] * mStrideY + ijk[2]];
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

	inline const openvdb::Coord dimensions() const {
		return openvdb::Coord(mRows,mCols,mSlices);
	}

	inline ValueT interpolate(float x, float y, float z) const {
		x = clamp(x,0.0f,(float)mRows);
		y = clamp(y,0.0f,(float)mCols);
		z = clamp(z,0.0f,(float)mSlices);
		int i = std::min((int)x,(int)mRows-1);
		int j = std::min((int)y,(int)mCols-1);
		int k = std::min((int)z,(int)mSlices-1);
		const RegularGrid<ValueT>& q=*this;
		return	(k+1-z)*(((i+1-x)*q(i,j,k)+(x-i)*q(i+1,j,k))*(j+1-y) + ((i+1-x)*q(i,j+1,k)+(x-i)*q(i+1,j+1,k))*(y-j)) +
				(z-k)*(((i+1-x)*q(i,j,k+1)+(x-i)*q(i+1,j,k+1))*(j+1-y) + ((i+1-x)*q(i,j+1,k+1)+(x-i)*q(i+1,j+1,k+1))*(y-j));
	}
	inline ValueT interpolate(const openvdb::Vec3d& pt) const{
		double x = clamp(pt[0],0.0,(double)mRows);
		double y = clamp(pt[1],0.0,(double)mCols);
		double z = clamp(pt[2],0.0,(double)mSlices);
		int i = std::min((int)x,(int)mRows-1);
		int j = std::min((int)y,(int)mCols-1);
		int k = std::min((int)z,(int)mSlices-1);
		const RegularGrid<ValueT>& q=*this;
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

struct plyVertex {
	float x[3];             // the usual 3-space position of a vertex
	float n[3];
	unsigned char red;
	unsigned char green;
	unsigned char blue;
	unsigned char alpha;
	plyVertex() {
		x[0] = 0;
		x[1] = 0;
		x[2] = 0;
		n[0] = 0;
		n[1] = 0;
		n[2] = 0;
		red = 0;
		green = 0;
		blue = 0;
		alpha = 0;
	}
};
struct plyParticle {
	float x[3];             // the usual 3-space position of a vertex
	float n[3];
	float radius;
	plyParticle() {
		x[0] = 0;
		x[1] = 0;
		x[2] = 0;
		n[0] = 0;
		n[1] = 0;
		n[2] = 0;
		radius=0.0f;
	}
};


typedef struct _plyFace {
	unsigned char nverts;    // number of vertex indices in list
	unsigned char nvels;    // number of vertex indices in list
	int *verts;              // vertex index list
	float* velocity;
	_plyFace() {
		nverts = 0;
		verts = NULL;
		velocity=NULL;
		nvels=3;
	}
} plyFace;
typedef struct _plyFaceTexutre {
	unsigned char nverts;    // number of vertex indices in list
	unsigned char nvels;    // number of vertex indices in list

	int *verts;              // vertex index list
	float* velocity;
	unsigned char uvcount;
	float* uvs;
	_plyFaceTexutre(){
		nverts = 0;
		uvs=NULL;
		verts = NULL;
		velocity=NULL;
		uvcount = 6;
		nvels=3;
	}
} plyFaceTexture;

class Exception:public std::exception{
protected:
	std::string message;
public:

	Exception(const char* str){
		message=str;
	}
	Exception(const std::string& str){
		message=str;
	}
	const char* what(){
		return message.c_str();
	}
};
std::string GetFileWithoutExtension(const std::string& file);
std::string GetFileNameWithoutExtension(const std::string& file);
std::string GetFileDirectoryPath(const std::string& file);
std::string GetFileName(const std::string& file);

int GetDirectoryListing(const std::string& dirName,
		std::vector<std::string>& files, const std::string& mask,
		const std::string& ext);
bool ReadImageFromFile(const std::string& file,
		std::vector<openvdb::math::Vec4<unsigned char>>& image, int& w, int& h);
bool WriteImageToFile(const std::string& file,
		const std::vector<RGBA>& image,
		const int w, const int h);
bool WriteImageToFile(const std::string& file,
		const std::vector<openvdb::math::Vec4s>& image,
		const int w, const int h);
bool WriteToRawFile(openvdb::FloatGrid::Ptr grid, const std::string& fileName);
bool WriteToRawFile(openvdb::tools::Dense<openvdb::Vec3s,openvdb::tools::MemoryLayout::LayoutZYX>& dense, const std::string& fileName);
bool WriteToRawFile(openvdb::tools::Dense<float,openvdb::tools::MemoryLayout::LayoutZYX>& grid, const std::string& fileName);
bool WriteToRawFile(openvdb::VectorGrid::Ptr grid, const std::string& fileName);
bool WriteToRawFile(openvdb::Int32Grid::Ptr grid, const std::string& fileName);
float DistanceToEdgeSqr(const openvdb::Vec3s& pt, const openvdb::Vec3s& pt1,
		const openvdb::Vec3s& pt2, openvdb::Vec3s* lastClosestSegmentPoint);
float DistanceToEdgeSqr(const openvdb::Vec3s& pt, const openvdb::Vec3s& pt1,
		const openvdb::Vec3s& pt2);
float DistanceToTriangleSqr(const openvdb::Vec3s& p, const openvdb::Vec3s& v0,
		const openvdb::Vec3s& v1, const openvdb::Vec3s& v2,
		openvdb::Vec3s* closestPoint);
float DistanceToQuadSqr(const openvdb::Vec3s& p, const openvdb::Vec3s& v0,
		const openvdb::Vec3s& v1, const openvdb::Vec3s& v2,
		const openvdb::Vec3s& v3, const openvdb::Vec3s& normal,
		openvdb::Vec3s* closestPoint);
const std::string ReadTextFile(const std::string& str);
float Angle(openvdb::Vec3s& v0, openvdb::Vec3s& v1, openvdb::Vec3s& v2);
openvdb::math::Mat3<float> CreateAxisAngle(openvdb::Vec3s axis, float angle);
}

#endif /* IMAGESCIUTIL_H_ */
