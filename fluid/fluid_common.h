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
#include "../SpringLevelSet.h"
#include "../ImageSciUtil.h"
#include "../Mesh.h"
#include <memory>
namespace imagesci {


enum class ObjectType {
	AIR = 0, FLUID = 1, WALL = 2
};
enum class ObjectMaterial {
	GLASS = 0, GRAY = 1, RED = 2
};

enum class ObjectShape {
	BOX = 0, SPHERE = 1, MESH = 2
};

struct SimulationObject {
public:
	ObjectType mType;
	ObjectShape mShape;
	ObjectMaterial mMaterial;
	bool mVisible;
	float mThickness;
	virtual bool inside(openvdb::Vec3f& pt){return false;}
	virtual bool insideShell(openvdb::Vec3f& pt){return false;}
	virtual ~SimulationObject(){};
	SimulationObject(ObjectShape shape):
		mShape(shape),mVisible(true),mThickness(0.0f),mType(ObjectType::AIR),mMaterial(ObjectMaterial::GRAY){

	}
};
struct SphereObject: public SimulationObject {
public:
	float mRadius;
	openvdb::Vec3f mCenter;
	SphereObject():SimulationObject(ObjectShape::SPHERE),mRadius(0),mCenter(){
	}
	virtual bool inside(openvdb::Vec3f& pt){
		float len = (pt-mCenter).length();
		if (len < mRadius) {
			return true;
		} else {
			return false;
		}
	}
	virtual bool insideShell(openvdb::Vec3f& pt){
		float len = (pt-mCenter).length();
		if (len < mRadius) {
			if(len < mRadius - mThickness) {
				return true;
			} else {
				return false;
			}
		} else {
			return false;
		}
	}
};
struct BoxObject: public SimulationObject {
public:
	openvdb::Vec3f mMin;
	openvdb::Vec3f mMax;
	BoxObject():SimulationObject(ObjectShape::BOX),mMin(),mMax(){
	}

	virtual bool inside(openvdb::Vec3f& pt){
		if (
				pt[0] > mMin[0] && pt[0] < mMax[0]&&
				pt[1] > mMin[1] && pt[1] < mMax[1]&&
				pt[2] > mMin[2] && pt[2] < mMax[2]) {
			return true;
		} else {
			return false;
		}
	}
	virtual bool insideShell(openvdb::Vec3f& pt){
		if (
				pt[0] > mMin[0] && pt[0] < mMax[0]&&
				pt[1] > mMin[1] && pt[1] < mMax[1]&&
				pt[2] > mMin[2] && pt[2] < mMax[2]) {
				if (
						pt[0] > mMin[0]+mThickness && pt[0] < mMax[0]-mThickness&&
						pt[1] > mMin[1]+mThickness && pt[1] < mMax[1]-mThickness&&
						pt[2] > mMin[2]+mThickness && pt[2] < mMax[2]-mThickness) {
				return false;
			} else {
				return true;
			}
		} else {
			return false;
		}
	}
};
struct MeshObject: public SimulationObject {
public:
	float mRadius;
	openvdb::Vec3f mCenter;
	RegularGrid<float>* mSignedLevelSet;
	MeshObject():SimulationObject(ObjectShape::MESH),mRadius(1.0f),mCenter(),mSignedLevelSet(nullptr){

	}
	virtual bool inside(openvdb::Vec3f& pt){
		BBoxd bbox=mSignedLevelSet->getBoundingBox();
		float localVoxelSize=mSignedLevelSet->rows();
		Vec3d lpt=localVoxelSize*((pt-mCenter)/(2*mRadius)+0.5f);
		if (mSignedLevelSet->interpolateWorld(lpt[0],lpt[1],lpt[2])<0.0f) {
			return true;
		} else {
			return false;
		}
	}
	virtual bool insideShell(openvdb::Vec3f& pt){
		BBoxd bbox=mSignedLevelSet->getBoundingBox();
		Vec3d dims=bbox.max()-bbox.min();
		float localVoxelSize=dims[0];
		Vec3d lpt=localVoxelSize*((pt-mCenter)*mRadius+0.5f);
		float val=mSignedLevelSet->interpolateWorld(lpt[0],lpt[1],lpt[2]);
		if (val>-mThickness&&val<0.0f) {
			return true;
		} else {
			return false;
		}
	}
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
#endif
