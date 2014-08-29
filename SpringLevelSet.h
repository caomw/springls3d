/*
 * Constellation.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef SPRINGLEVELSET_H_
#define SPRINGLEVELSET_H_
#include <openvdb/util/NullInterrupter.h>

#include <tbb/parallel_for.h>
#include <vector>
#include <list>
#include <iostream>
#include "Mesh.h"
#include "ImageSciUtil.h"
#include "AdvectionForce.h"
#undef OPENVDB_REQUIRE_VERSION_NAME
namespace imagesci{

class Constellation;
struct Springl;
struct SpringlNeighbor{
public:
	openvdb::Index32 springlId;
	int edgeId;
	float distance;
	SpringlNeighbor(openvdb::Index32 id=0,int nbr=-1,float _distance=0):springlId(id),edgeId(nbr),distance(_distance){
	}
};
std::ostream& operator<<(std::ostream& ostr, const SpringlNeighbor& classname);
typedef std::vector<std::list<SpringlNeighbor>> NearestNeighborMap;
class SpringLevelSet {
public:
	static const float NEAREST_NEIGHBOR_RANGE;//voxel units
	static const int MAX_NEAREST_NEIGHBORS;
	static const float PARTICLE_RADIUS;
	static const float MAX_VEXT;
	static const float FILL_DISTANCE;
	static const float SHARPNESS;
	static const float SPRING_CONSTANT;
	static const float RELAX_TIMESTEP;
	static const float MAX_ANGLE_TOLERANCE;
	static const float MIN_ANGLE_TOLERANCE;
	static const float MIN_AREA;

	openvdb::FloatGrid::Ptr signedLevelSet;
	openvdb::FloatGrid::Ptr unsignedLevelSet;
	openvdb::VectorGrid::Ptr gradient;
	openvdb::Int32Grid::Ptr springlIndexGrid;
	boost::shared_ptr<Mesh> isoSurface;
	boost::shared_ptr<Constellation> constellation;
	NearestNeighborMap nearestNeighbors;
	std::vector<openvdb::Vec3s> vertexDisplacement;
	std::vector<openvdb::Vec3s> particleDisplacement;
	Springl& GetSpringl(const openvdb::Index32 id);
	openvdb::Vec3s& GetParticle(const openvdb::Index32 id);
	openvdb::Vec3s& GetParticleNormal(const openvdb::Index32 id);
	openvdb::Vec3s& GetSpringlVertex(const openvdb::Index32 id,const int i);
	openvdb::Vec3s& GetSpringlVertex(const openvdb::Index32 gid);
	std::list<SpringlNeighbor>& GetNearestNeighbors(openvdb::Index32 id,int8_t e);
	void draw(bool colorEnabled=false,bool wireframe=true,bool particles=false,bool particleNormals=false);
	void clean();
	void fill();
	void updateGradient();
	void updateUnsignedLevelSet();
	void relax(int iters=10);
	void updateNearestNeighbors(bool threaded=true);
	void create(Mesh* mesh);
	SpringLevelSet(){}
	~SpringLevelSet(){}
};

struct Springl {
	private:
		Mesh* mesh;
	public:
		openvdb::Index32 id;
		openvdb::Index32 offset;

		openvdb::Vec3s& normal() const {return mesh->particleNormals[id];}
		openvdb::Vec3s& particle() const {return mesh->particles[id];}
		openvdb::Vec3s& operator[](size_t idx){return mesh->vertexes[offset+idx];}
		const openvdb::Vec3s& operator[](size_t idx) const {return mesh->vertexes[offset+idx];}
		Springl():id(0),offset(0),mesh(NULL){

		}
		Springl(Mesh* _mesh):id(0),offset(0),mesh(_mesh){
		}

		int size() const;
		float area() const;
		float distanceToFace(const openvdb::Vec3s& pt);
		float distanceToFaceSqr(const openvdb::Vec3s& pt);
		float distanceToParticle(const openvdb::Vec3s& pt);
		float distanceToParticleSqr(const openvdb::Vec3s& pt);
		float distanceEdgeSqr(const openvdb::Vec3s& pt,int e);
		float distanceEdge(const openvdb::Vec3s& pt,int e);
		openvdb::Vec3s computeCentroid() const;
		openvdb::Vec3s computeNormal(const float eps=1E-6f) const;
		~Springl(){
		}
};


class Constellation {
public:
		Mesh storage;
		std::vector<Springl> springls;
		inline openvdb::BBoxd GetBBox(){
			return storage.GetBBox();
		}
		Constellation(Mesh* mesh);

		inline void draw(bool colorEnabled,bool wireframe,bool particles,bool particleNormals){
			storage.draw(colorEnabled,wireframe,particles,particleNormals);
		}
		inline void updateGL(){
			storage.updateGL();
		}
		virtual ~Constellation(){}
		inline size_t getNumSpringls() const {return springls.size();}
		inline size_t getNumVertexes() const {return storage.vertexes.size();}

		Springl& operator[](size_t idx)
	    {
	    	return springls[idx];
	    }
		const Springl& operator[](size_t idx) const
	    {
	    	return springls[idx];
	    }
};

class SpringlRange
{
public:
    class Iterator
    {
    public:
        Iterator(const SpringlRange& range, size_t pos): mRange(range), mPos(pos)
        {
            assert(this->isValid());
        }
        /*
        Iterator& operator=(const Iterator& other)
        {
            mRange = other.mRange;
            mPos = other.mPos;
            return *this;
        }
        */
        /// Advance to the next leaf node.
        Iterator& operator++() { ++mPos; return *this; }
        /// Return a reference to the leaf node to which this iterator is pointing.
        Springl& operator*() const { return mRange.mConstellation[mPos]; }
        /// Return a pointer to the leaf node to which this iterator is pointing.
        Springl* operator->() const { return &(this->operator*()); }

        /// Return the index into the leaf array of the current leaf node.
        size_t pos() const { return mPos; }
        bool isValid() const { return mPos>=mRange.mBegin && mPos<=mRange.mEnd; }
        /// Return @c true if this iterator is not yet exhausted.
        bool test() const { return mPos < mRange.mEnd; }
        /// Return @c true if this iterator is not yet exhausted.
        operator bool() const { return this->test(); }
        /// Return @c true if this iterator is exhausted.
        bool empty() const { return !this->test(); }
        bool operator!=(const Iterator& other) const
        {
            return (mPos != other.mPos) || (&mRange != &other.mRange);
        }
        bool operator==(const Iterator& other) const { return !(*this != other); }
        const SpringlRange& leafRange() const { return mRange; }

    protected:
        const SpringlRange& mRange;
        size_t mPos;
    };// end Iterator

    SpringlRange(size_t begin, size_t end, Constellation& constellation, size_t grainSize=1):
        mEnd(end), mBegin(begin), mGrainSize(grainSize), mConstellation(constellation) {}

    SpringlRange(Constellation& constellation, size_t grainSize=1):
        mEnd(constellation.getNumSpringls()), mBegin(0), mGrainSize(grainSize), mConstellation(constellation) {}

    Iterator begin() const {return Iterator(*this, mBegin);}

    Iterator end() const {return Iterator(*this, mEnd);}

    size_t size() const { return mEnd - mBegin; }

    size_t grainsize() const { return mGrainSize; }

    const Constellation& getConstellation() const { return mConstellation; }

    bool empty() const {return !(mBegin < mEnd);}

    bool is_divisible() const {return mGrainSize < this->size();}

    SpringlRange(SpringlRange& r, tbb::split):
        mEnd(r.mEnd), mBegin(doSplit(r)), mGrainSize(r.mGrainSize),
          mConstellation(r.mConstellation) {}
    Constellation& mConstellation;
private:
    size_t mEnd, mBegin, mGrainSize;


    static size_t doSplit(SpringlRange& r)
    {
        assert(r.is_divisible());
        size_t middle = r.mBegin + (r.mEnd - r.mBegin) / 2u;
        r.mEnd = middle;
        return middle;
    }
};// end of SpringlRange

	template<typename OperatorT,typename InterruptT = openvdb::util::NullInterrupter>
	class ConstellationOperator
	{
		typedef Springl SpringlType;
	public:
		SpringLevelSet& mGrid;
		ConstellationOperator(SpringLevelSet& grid,InterruptT* _interrupt):mGrid(grid),mInterrupt(_interrupt){
		}

		virtual ~ConstellationOperator() {}
		void process(bool threaded = true)
		{
			if (mInterrupt) mInterrupt->start("Processing springls");
			OperatorT::init(mGrid);
			SpringlRange range(*mGrid.constellation);
			if (threaded) {
				tbb::parallel_for(range, *this);
			} else {
				(*this)(range);
			}

			if (mInterrupt) mInterrupt->end();
		}

		/// @note Never call this public method directly - it is called by
		/// TBB threads only!
		void operator()(const SpringlRange& range) const
		{
			if (openvdb::util::wasInterrupted(mInterrupt)) tbb::task::self().cancel_group_execution();
			for (typename SpringlRange::Iterator springl=range.begin(); springl; ++springl) {
				OperatorT::result(*springl,mGrid);
			}
		}

	protected:
		InterruptT*         mInterrupt;

	};

	struct NearestNeighborOperation
	{
	private:

	public:
		static void init(SpringLevelSet& mGrid);
	    static void result(Springl& springl,SpringLevelSet& mGrid);
	};
	struct RelaxOperation
	{
	private:

	public:
		static void init(SpringLevelSet& mGrid);
	    static void result(Springl& springl,SpringLevelSet& mGrid);
	};
	struct AdvectVertexOperation
	{
	private:

	public:
		static void init(SpringLevelSet& mGrid);
	    static void result(Springl& springl,SpringLevelSet& mGrid);
	};
	// end of ConstellationOperator class
	/// @brief Compute the gradient of a scalar grid.
	template<typename InterruptT = openvdb::util::NullInterrupter>
	class NearestNeighbors
	{
	public:
	    NearestNeighbors(
	    			SpringLevelSet& grid,
	    			InterruptT* interrupt = NULL):mGrid(grid),mInterrupt(interrupt)
	    {
	    }
	    void process(bool threaded = true)
	    {
        	typedef NearestNeighborOperation OpT;
	    	ConstellationOperator<OpT,InterruptT> op(mGrid,mInterrupt);
        	op.process(threaded);
	    }
	    SpringLevelSet& mGrid;
	    InterruptT*          mInterrupt;
	}; // end of Gradient class

	template<typename InterruptT = openvdb::util::NullInterrupter>
	class Relax
	{
	public:
	    Relax(
	    			SpringLevelSet& grid,
	    			InterruptT* interrupt = NULL):mGrid(grid),mInterrupt(interrupt)
	    {
	    }
	    void process(bool threaded = true)
	    {
        	typedef RelaxOperation OpT;
	    	ConstellationOperator<OpT,InterruptT> op(mGrid,mInterrupt);
        	op.process(threaded);
	    }
	    SpringLevelSet& mGrid;
	    InterruptT*          mInterrupt;
	};
	template<typename InterruptT = openvdb::util::NullInterrupter>
	class AdvectVertex
	{
	public:
		AdvectVertex(
	    			SpringLevelSet& grid,
	    			InterruptT* interrupt = NULL):mGrid(grid),mInterrupt(interrupt)
	    {
	    }
	    void process(bool threaded = true)
	    {
        	typedef AdvectVertexOperation OpT;
	    	ConstellationOperator<OpT,InterruptT> op(mGrid,mInterrupt);
        	op.process(threaded);
	    }
	    SpringLevelSet& mGrid;
	    InterruptT*          mInterrupt;
	};
}

#endif /* CONSTELLATION_H_ */
