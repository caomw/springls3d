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

#ifndef FLUIDTRACKINGFIELD_H_
#define FLUIDTRACKINGFIELD_H_

#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/DenseSparseTools.h>
#include "fluid_common.h"
#undef OPENVDB_REQUIRE_VERSION_NAME
namespace imagesci {

template <typename ScalarT = float>
class FluidTrackingField
{
protected:
	openvdb::Vec3d mTwistPosition;
public:
    typedef ScalarT             ScalarType;
    typedef openvdb::math::Vec3<ScalarT> VectorType;
    RegularGrid<ScalarT>& mLevelSet;
    FluidTrackingField(RegularGrid<float>& levelSet):mLevelSet(levelSet){
    }
    /// @return const reference to the identity transfrom between world and index space
    /// @note Use this method to determine if a client grid is
    /// aligned with the coordinate space of this velocity field
    const openvdb::math::Transform& transform() const { return mLevelSet.transform(); }

    /// @return the velocity in world units, evaluated at the world
    /// position xyz and at the specified time
    inline VectorType operator()(const openvdb::Vec3d& pt, ScalarType time) const{
    	openvdb::Vec3d xyz=transform().worldToIndex(pt);
        ScalarT v111=mLevelSet.interpolate(xyz[0],xyz[1],xyz[2]);

        ScalarT v110=mLevelSet.interpolate(xyz[0],xyz[1],xyz[2]-1.0f);
        ScalarT v112=mLevelSet.interpolate(xyz[0],xyz[1],xyz[2]+1.0f);

        ScalarT v101=mLevelSet.interpolate(xyz[0],xyz[1]-1.0f,xyz[2]);
        ScalarT v121=mLevelSet.interpolate(xyz[0],xyz[1]+1.0f,xyz[2]);

        ScalarT v011=mLevelSet.interpolate(xyz[0]-1.0f,xyz[1],xyz[2]);
        ScalarT v211=mLevelSet.interpolate(xyz[0]+1.0f,xyz[1],xyz[2]);

        VectorType v(v211-v011,v121-v101,v112-v110);
        v.normalize();
        v*=-v111/openvdb::LEVEL_SET_HALF_WIDTH;
        return v;
    }
    /// @return the velocity at the coordinate space position ijk
    inline VectorType operator() (const openvdb::Coord& xyz, ScalarType time) const
    {
        ScalarT v111=mLevelSet.interpolate(xyz[0],xyz[1],xyz[2]);

        ScalarT v110=mLevelSet.interpolate(xyz[0],xyz[1],xyz[2]-1.0f);
        ScalarT v112=mLevelSet.interpolate(xyz[0],xyz[1],xyz[2]+1.0f);

        ScalarT v101=mLevelSet.interpolate(xyz[0],xyz[1]-1.0f,xyz[2]);
        ScalarT v121=mLevelSet.interpolate(xyz[0],xyz[1]+1.0f,xyz[2]);

        ScalarT v011=mLevelSet.interpolate(xyz[0]-1.0f,xyz[1],xyz[2]);
        ScalarT v211=mLevelSet.interpolate(xyz[0]+1.0f,xyz[1],xyz[2]);

        VectorType v(v211-v011,v121-v101,v112-v110);
        v.normalize();
        v*=-v111/openvdb::LEVEL_SET_HALF_WIDTH;
        return v;
    }
}; // end of TwistField

} /* namespace imagesci */
#endif /* FLUIDTRACKINGFIELD_H_ */
