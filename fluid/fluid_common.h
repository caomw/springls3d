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
	virtual float signedDistance(openvdb::Vec3f& pt){return 0;}
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
	float mVoxelSize;
	SphereObject():SimulationObject(ObjectShape::SPHERE),mRadius(0),mCenter(){
	}
	virtual float signedDistance(openvdb::Vec3f& pt);
	virtual bool inside(openvdb::Vec3f& pt);
	virtual bool insideShell(openvdb::Vec3f& pt);
};
struct BoxObject: public SimulationObject {
public:
	openvdb::Vec3f mMin;
	openvdb::Vec3f mMax;
	float mVoxelSize;
	BoxObject():SimulationObject(ObjectShape::BOX),mMin(),mMax(),mVoxelSize(1.0f){
	}

	virtual bool inside(openvdb::Vec3f& pt);
	virtual float signedDistance(openvdb::Vec3f& pt);
	virtual bool insideShell(openvdb::Vec3f& pt);
};
struct MeshObject: public SimulationObject {
public:
	float mRadius;
	float mVoxelSize;
	openvdb::Vec3f mCenter;
	RegularGrid<float>* mSignedLevelSet;
	MeshObject():SimulationObject(ObjectShape::MESH),mVoxelSize(1.0f),mRadius(1.0f),mCenter(),mSignedLevelSet(nullptr){

	}
	virtual float signedDistance(openvdb::Vec3f& pt);
	virtual bool inside(openvdb::Vec3f& pt);
	virtual bool insideShell(openvdb::Vec3f& pt);
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
