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

#ifndef FLUIDVELOCITYFIELD_H_
#define FLUIDVELOCITYFIELD_H_

#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/DenseSparseTools.h>
#include "fluid_common.h"
#undef OPENVDB_REQUIRE_VERSION_NAME
namespace imagesci {

template <typename ScalarT = float>
class FluidVelocityField
{
protected:
	openvdb::Vec3d mTwistPosition;
public:
    typedef ScalarT             ScalarType;
    typedef openvdb::math::Vec3<ScalarT> VectorType;
    fluid::MACGrid<ScalarT>& mGrid;
    double mGridRatio;
    double mGridRatioInverse;
    RegularGrid<openvdb::Vec3s>& mDenseMap;
    FluidVelocityField(fluid::MACGrid<ScalarT>& grid,RegularGrid<openvdb::Vec3s>& denseMap,float gridRatio=0.5):mDenseMap(denseMap),mGridRatio(gridRatio),mGridRatioInverse(1.0/gridRatio),mGrid(grid){
    }
    void update(const FloatGrid& levelSet){
    	openvdb::VectorGrid::Ptr mClosestPoints=openvdb::tools::cpt(levelSet);
    	mDenseMap.fill(Vec3s(0.0f));
    	openvdb::tools::copyToDense(*mClosestPoints, mDenseMap);
    }
    /// @return const reference to the identity transfrom between world and index space
    /// @note Use this method to determine if a client grid is
    /// aligned with the coordinate space of this velocity field
    const openvdb::math::Transform& transform() const { return mGrid.transform(); }

    /// @return the velocity in world units, evaluated at the world
    /// position xyz and at the specified time
    inline VectorType operator()(const openvdb::Vec3d& pt, ScalarType time) const{
    	openvdb::Vec3d xyz=transform().worldToIndex(pt);
        openvdb::Vec3s cpt=mDenseMap.interpolate(xyz);
    	return mGrid.interpolate(mGridRatio*transform().indexToWorld(cpt))*mGridRatioInverse;
    }
    /// @return the velocity at the coordinate space position ijk
    inline VectorType operator() (const openvdb::Coord& ijk, ScalarType time) const
    {
    	Vec3s cpt=mDenseMap.getValue(ijk);
    	return mGrid.interpolate(mGridRatio*transform().indexToWorld(cpt))*mGridRatioInverse;
    }
}; // end of TwistField

} /* namespace imagesci */
#endif /* FLUIDVELOCITYFIELD_H_ */
