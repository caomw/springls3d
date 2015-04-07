/*
 * fluid_common.cc
 *
 *  Created on: Apr 6, 2015
 *      Author: blake
 */

#include "fluid_common.h"

namespace imagesci{
bool BoxObject::inside(openvdb::Vec3f& pt){
		float delta=-0.25f*mVoxelSize;
		//std::cout<<"BOX "<<mMin/mVoxelSize<<" "<<mMax/mVoxelSize<<" "<<delta/mVoxelSize<<" "<<mVoxelSize<<std::endl;
		if (
				pt[0] > mMin[0] +delta && pt[0] < mMax[0]-delta&&
				pt[1] > mMin[1] +delta && pt[1] < mMax[1]-delta&&
				pt[2] > mMin[2] +delta && pt[2] < mMax[2]-delta) {
			return true;
		} else {
			return false;
		}
	}
float BoxObject::signedDistance(openvdb::Vec3f& pt){
		if (
				pt[0] > mMin[0] && pt[0] < mMax[0]&&
				pt[1] > mMin[1] && pt[1] < mMax[1]&&
				pt[2] > mMin[2] && pt[2] < mMax[2]) {
			float dx=std::min(pt[0]-mMax[0],mMin[0]-pt[0]);
			float dy=std::min(pt[1]-mMax[1],mMin[1]-pt[1]);
			float dz=std::min(pt[2]-mMax[2],mMin[2]-pt[2]);
			return std::min(std::min(dx,dy),dz);
		} else {
			float dx=std::max(pt[0]-mMax[0],mMin[0]-pt[0]);
			float dy=std::max(pt[1]-mMax[1],mMin[1]-pt[1]);
			float dz=std::max(pt[2]-mMax[2],mMin[2]-pt[2]);
			return std::max(std::max(dx,dy),dz);
		}
	}
bool BoxObject::insideShell(openvdb::Vec3f& pt){
		float delta=-0.25f*mVoxelSize;
		if (
				pt[0] > mMin[0]+delta && pt[0] < mMax[0]-delta&&
				pt[1] > mMin[1]+delta && pt[1] < mMax[1]-delta&&
				pt[2] > mMin[2]+delta && pt[2] < mMax[2]-delta) {
				if (
						pt[0] > mMin[0]+mThickness+delta && pt[0] < mMax[0]-mThickness-delta&&
						pt[1] > mMin[1]+mThickness+delta && pt[1] < mMax[1]-mThickness-delta&&
						pt[2] > mMin[2]+mThickness+delta && pt[2] < mMax[2]-mThickness-delta) {
				return false;
			} else {
				return true;
			}
		} else {
			return false;
		}
	}
float MeshObject::signedDistance(openvdb::Vec3f& pt){
	BBoxd bbox=mSignedLevelSet->getBoundingBox();
	float localVoxelSize=mSignedLevelSet->rows();
	Vec3d lpt=localVoxelSize*((pt-mCenter)/(2*mRadius)+0.5f);
	return (mSignedLevelSet->interpolateWorld(lpt[0],lpt[1],lpt[2]))*mVoxelSize*(2*mRadius);
}
bool MeshObject::inside(openvdb::Vec3f& pt){
	BBoxd bbox=mSignedLevelSet->getBoundingBox();
	float localVoxelSize=mSignedLevelSet->rows();
	Vec3d lpt=localVoxelSize*((pt-mCenter)/(2*mRadius)+0.5f);
	if (mSignedLevelSet->interpolateWorld(lpt[0],lpt[1],lpt[2])<-0.5f) {
		return true;
	} else {
		return false;
	}
}
bool MeshObject::insideShell(openvdb::Vec3f& pt){
	BBoxd bbox=mSignedLevelSet->getBoundingBox();
	Vec3d dims=bbox.max()-bbox.min();
	float localVoxelSize=dims[0];
	Vec3d lpt=localVoxelSize*((pt-mCenter)*mRadius+0.5f);
	float val=mSignedLevelSet->interpolateWorld(lpt[0],lpt[1],lpt[2]);
	if (val>-mThickness-0.5f&&val<-0.5f) {
		return true;
	} else {
		return false;
	}
}
float SphereObject::signedDistance(openvdb::Vec3f& pt){
	float len = (pt-mCenter).length();
	return len-mRadius;
}
bool SphereObject::inside(openvdb::Vec3f& pt){
	float len = (pt-mCenter).length();
	if (len < mRadius-0.25f*mVoxelSize) {
		return true;
	} else {
		return false;
	}
}
bool SphereObject::insideShell(openvdb::Vec3f& pt){
	float len = (pt-mCenter).length();
	if (len < mRadius) {
		if(len < mRadius - mThickness-0.25f*mVoxelSize) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}
}

