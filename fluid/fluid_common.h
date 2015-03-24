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
	openvdb::math::Transform::Ptr mTransform;
public:

	MACGrid(const openvdb::Coord& dim, float voxelSize,ValueT value=0.0) :
			mX(dim[0] + 1, dim[1], dim[2], voxelSize,value),
			mY(dim[0], dim[1] + 1, dim[2], voxelSize ,value),
			mZ(dim[0], dim[1], dim[2] + 1, voxelSize ,value),mRows(dim[0]),mCols(dim[1]),mSlices(dim[2]),mVoxelSize(voxelSize) {
		mTransform=openvdb::math::Transform::createLinearTransform(mVoxelSize);
	}
	RegularGrid<ValueT>& operator[](size_t i) {
		return (&mX)[i];
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
	inline openvdb::math::Vec3<ValueT> interpolate(const openvdb::Vec3s& p) const {
		openvdb::math::Vec3<ValueT> u;
		u[0] = mX.interpolate(mRows*p[0], mCols*p[1]-0.5, mSlices*p[2]-0.5);
		u[1] = mY.interpolate(mRows*p[0]-0.5, mCols*p[1], mSlices*p[2]-0.5);
		u[2] = mZ.interpolate(mRows*p[0]-0.5, mCols*p[1]-0.5, mSlices*p[2]);
		return u;
	}
	inline openvdb::math::Vec3<ValueT> maxInterpolate(const openvdb::Vec3f& position,float radius){
		openvdb::math::Vec3<ValueT> values[15];
		const float n3=1.0f/std::sqrt(3.0f);
		MACGrid<ValueT>& grid=*this;
		values[0]=grid.interpolate(position);
		values[1]=grid.interpolate(position+openvdb::Vec3f(+radius,0,0));
		values[2]=grid.interpolate(position+openvdb::Vec3f(-radius,0,0));
		values[3]=grid.interpolate(position+openvdb::Vec3f(0,0,+radius));
		values[4]=grid.interpolate(position+openvdb::Vec3f(0,0,-radius));
		values[5]=grid.interpolate(position+openvdb::Vec3f(0,+radius,0));
		values[6]=grid.interpolate(position+openvdb::Vec3f(0,-radius,0));

		values[7 ]=grid.interpolate(position+openvdb::Vec3f(+radius*n3,+radius*n3,+radius*n3));
		values[8 ]=grid.interpolate(position+openvdb::Vec3f(+radius*n3,-radius*n3,+radius*n3));
		values[9 ]=grid.interpolate(position+openvdb::Vec3f(-radius*n3,+radius*n3,+radius*n3));
		values[10]=grid.interpolate(position+openvdb::Vec3f(-radius*n3,-radius*n3,+radius*n3));
		values[11]=grid.interpolate(position+openvdb::Vec3f(+radius*n3,+radius*n3,-radius*n3));
		values[12]=grid.interpolate(position+openvdb::Vec3f(+radius*n3,-radius*n3,-radius*n3));
		values[13]=grid.interpolate(position+openvdb::Vec3f(-radius*n3,+radius*n3,-radius*n3));
		values[14]=grid.interpolate(position+openvdb::Vec3f(-radius*n3,-radius*n3,-radius*n3));

		ValueT maxVal=std::numeric_limits<ValueT>::min();
		openvdb::math::Vec3<ValueT> final=values[0];
		for(int i=0;i<15;i++){
			ValueT lsqr=values[i].lengthSqr();
			if(lsqr>maxVal){
				maxVal=lsqr;
				final=values[i];
			}
		}
		return final;
	}
};
inline bool WriteToRawFile(MACGrid<float>& mac, const std::string& fileName) {
	bool r1 = WriteToRawFile(mac[0], fileName + "_x");
	bool r2 = WriteToRawFile(mac[1], fileName + "_y");
	bool r3 = WriteToRawFile(mac[2], fileName + "_z");
	return (r1 && r2 && r3);
}

enum class ObjectType {
	AIR = 0, FLUID = 1, WALL = 2
};
enum class ObjectMaterial {
	GLASS = 0, GRAY = 1, RED = 2
};
enum class ObjectShape {
	BOX = 0, SPHERE = 1
};
struct SimulationObject {
	ObjectType type;
	ObjectShape shape;
	ObjectMaterial material;
	bool mVisible;
	float mRadius;
	openvdb::Vec3f mCenter;
	openvdb::Vec3f mBounds[2];
};

struct FluidParticle {
	openvdb::Vec3f mLocation;
	openvdb::Vec3f mVelocity;
	openvdb::Vec3f mNormal;
	ObjectType mObjectType;
	//char mVisible;
	bool mRemoveIndicator;
	openvdb::Vec3f mTmp[2];
	float mMass;
	float mDensity;
};
typedef std::unique_ptr<FluidParticle> ParticlePtr;
}
}
#endif
