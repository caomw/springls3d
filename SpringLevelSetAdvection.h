#ifndef SPRINGLEVELSETADVECTION_H_
#define SPRINGLEVELSETADVECTION_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetAdvect.h>
namespace imagesci{
template<typename FieldT = openvdb::tools::EnrightField<float>,
         typename InterruptT = openvdb::util::NullInterrupter>
class SpringLevelSetAdvection
{
public:
    /// Main constructor
    SpringLevelSetAdvection(SpringLevelSet& grid, const FieldT& field, InterruptT* interrupt = NULL):
        mGrid(grid), mField(field),
        mTemporalScheme(SpringlTemporalIntegrationScheme::TVD_RK4) {}
    /// @return the temporal integration scheme
    SpringlTemporalIntegrationScheme getTemporalScheme() const { return mTemporalScheme; }
    /// @brief Set the spatial finite difference scheme
    void setTemporalScheme(SpringlTemporalIntegrationScheme scheme) { mTemporalScheme = scheme; }
    size_t advect(double time0, double time1){
    	AdvectVertex<FieldT> advect(mGrid,mField,time0,time1);
    	advect.process();
    	return 0;
    }
    SpringLevelSet& mGrid;
    const FieldT&                   mField;
    SpringlTemporalIntegrationScheme mTemporalScheme;

    // disallow copy by assignment
    void operator=(const SpringLevelSetAdvection& other) {}

};//end of LevelSetAdvection
}
#endif
