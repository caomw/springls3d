/*
 *  common.h
 *  flip3D
 *
 */
#ifndef _FLUIDCOMMON_H
#define _FLUIDCOMMON_H
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>
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
	ValueT& operator[](size_t k) {
		return mPtr[k];
	}
	const ValueT& operator[](size_t k) const {
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
	Offset1D<ValueT> operator[](size_t j) {
		return Offset1D<ValueT>(&mPtr[strideY * j]);
	}
	const Offset1D<ValueT> operator[](size_t j) const {
		return Offset1D<ValueT>(&mPtr[strideY * j]);
	}
};

template<typename ValueT> class RegularGrid: public openvdb::tools::Dense<ValueT,
openvdb::tools::MemoryLayout::LayoutZYX> {
private:
	ValueT* mPtr;
	size_t mStrideX;
	size_t mStrideY;
public:
	RegularGrid(const openvdb::Coord& dim, const openvdb::Coord& min, ValueT value) :
		openvdb::tools::Dense<ValueT, openvdb::tools::MemoryLayout::LayoutZYX>(dim, min) {
		this->fill(value);
		mPtr = this->data();
		mStrideX = this->xStride();
		mStrideY = this->yStride();
	}
	RegularGrid(int rows,int cols,int slices) :
		openvdb::tools::Dense<ValueT, openvdb::tools::MemoryLayout::LayoutZYX>(openvdb::Coord(rows,cols,slices),openvdb::Coord(0)) {
		mPtr = this->data();
		mStrideX = this->xStride();
		mStrideY = this->yStride();
	}
	ValueT& operator()(size_t i, size_t j, size_t k) {
		return mPtr[i * mStrideX + j * mStrideY + k];
	}
	const ValueT& operator()(size_t i, size_t j, size_t k) const {
		return mPtr[i * mStrideX + j * mStrideY + k];
	}
	Offset2D<ValueT> operator[](size_t i) {
		return Offset2D<ValueT>(&mPtr[i * mStrideX], mStrideY);
	}
	const Offset2D<ValueT> operator[](size_t i) const {
		return Offset2D<ValueT>(&mPtr[i * mStrideX], mStrideY);
	}
	const size_t size() const {
		return this->bbox().volume();
	}
	void copyTo(RegularGrid<ValueT>& out){
		ValueT* src=this->data();
		ValueT* dest=out.data();
		memcpy(dest,src,sizeof(ValueT)*size());
	}
	void add(RegularGrid<ValueT>& out){
		ValueT* src=this->data();
		ValueT* dest=out.data();
		size_t N=size();
		for(size_t n=0;n<N;n++){
			*src+=*dest;
			src++;
			dest++;
		}
	}
	void subtract(RegularGrid<ValueT>& out){
		ValueT* src=this->data();
		ValueT* dest=out.data();
		size_t N=size();
		for(size_t n=0;n<N;n++){
			*src-=*dest;
			src++;
			dest++;
		}
	}
	void subtractFrom(RegularGrid<ValueT>& out){
		ValueT* src=this->data();
		ValueT* dest=out.data();
		size_t N=size();
		for(size_t n=0;n<N;n++){
			*src=*dest-*src;
			src++;
			dest++;
		}
	}
};
template<typename ValueT> struct MACGrid {
public:
	RegularGrid<ValueT> mX, mY, mZ;
	MACGrid(const openvdb::Coord& dim, const openvdb::Coord& min, ValueT value) :
					mX(openvdb::Coord(dim[0]+1,dim[1],dim[2]), min, value),
					mY(openvdb::Coord(dim[0],dim[1]+1,dim[2]), min, value),
					mZ(openvdb::Coord(dim[0],dim[1],dim[2]+1), min, value) {
	}
	RegularGrid<ValueT>& operator[](size_t i) {
		return (&mX)[i];
	}

};
enum ObjectType {
	AIR = 0, FLUID = 1, WALL = 2
};
enum MaterialType {
	GLASS = 0, GRAY = 1, RED = 2
};
enum ObjectShape {
	BOX= 0, SPHERE = 1
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
