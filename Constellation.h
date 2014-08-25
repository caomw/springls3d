/*
 * Constellation.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef CONSTELLATION_H_
#define CONSTELLATION_H_

#include "Mesh.h"
#include <openvdb/util/Util.h>
#include <openvdb/util/NullInterrupter.h>
#include <openvdb/math/Stencils.h>
#include <tbb/parallel_for.h>
#include <vector>
#include <list>
typedef std::vector<std::list<openvdb::Index32>> NearestNeighborMap;
namespace imagesci{
template<typename Description> class Constellation;
class SpringlGrid {
public:
	openvdb::FloatGrid::Ptr signedLevelSet;
	openvdb::FloatGrid::Ptr unsignedLevelSet;
	openvdb::VectorGrid::Ptr gradient;
	openvdb::Int32Grid::Ptr springlIndexGrid;
	boost::shared_ptr<Mesh> isoSurface;
	boost::shared_ptr<Constellation<openvdb::Int32>> constellation;
	NearestNeighborMap nearestNeighbors;
	void draw(bool colorEnabled);
	void updateGradient();
	void updateUnsignedLevelSet();
	void updateNearestNeighbors(bool threaded=true);
	bool create(const Mesh& mesh,openvdb::math::Transform::Ptr& transform);
	SpringlGrid();
	~SpringlGrid();
};
template<typename Description> struct SpringlBase {
	private:
		size_t K;
	public:
		openvdb::Index32 id;
		openvdb::Vec3s* vertexes;//At most 4! So it's either a triangle or quad, NOTHING ELSE!
		openvdb::Vec3s* particle;
		openvdb::Vec3s* normal;
		Description description;
		openvdb::Vec3s& operator[](size_t idx){return (vertexes[idx]);}
		const openvdb::Vec3s& operator[](size_t idx) const {return vertexes[idx];}
		void set(int index,openvdb::Vec3s ptr){
			vertexes[index]=ptr;
		}
		openvdb::Vec3s get(int index){
			return vertexes[index];
		}
		SpringlBase():vertexes(NULL),K(0),id(0),particle(NULL),normal(NULL){

		}
		SpringlBase(openvdb::Vec3s* ptr,size_t k):vertexes(ptr),K(k),id(0),particle(NULL),normal(NULL){

		}
		size_t size() const {return K;}
		openvdb::Vec3s computeCentroid() const{
			openvdb::Vec3s centroid=openvdb::Vec3s(0.0f,0.0f,0.0f);
			for(int k=0;k<K;k++){
				centroid+=(*this)[k];
			}
			centroid=(1.0/K)*centroid;
			return centroid;
		}
		openvdb::Vec3s computeNormal(const float eps=1E-6f) const{
			openvdb::Vec3s norm;
			norm=(vertexes[2]-vertexes[0]).cross(vertexes[1]-vertexes[0]);
			norm.normalize(eps);
			return norm;
		}
		~SpringlBase(){
		}
};
template<typename Description,size_t K> struct Springl : public SpringlBase<Description>{
	public:
		Springl(openvdb::Vec3s* ptr):SpringlBase<Description>(ptr,K){
		}
};


template<typename Description> class Constellation {
protected:
		Mesh storage;
		std::vector<SpringlBase<Description>> springls;
public:
		inline openvdb::BBoxd GetBBox(){
			return storage.GetBBox();
		}
		Constellation(Mesh* mesh):storage(){
			size_t faceCount=mesh->faces.size();
			size_t counter=0;
			size_t pcounter=0;
			springls.reserve(faceCount);
			storage.faces.reserve(faceCount);
			storage.vertexes.resize(mesh->indexes.size());
			storage.particles.resize(faceCount);
			storage.particleNormals.resize(faceCount);
			storage.normals.resize(mesh->indexes.size());
			for(openvdb::Vec4I face:mesh->faces){
				if(face[3]!=openvdb::util::INVALID_IDX){
					storage.meshType=Mesh::PrimitiveType::QUADS;
					storage.faces.push_back(openvdb::Vec4I(counter,counter+1,counter+2,counter+3));
					Springl<Description,4> springl(&(storage.vertexes[counter]));
					storage.vertexes[counter++]=mesh->vertexes[face[0]];
					storage.vertexes[counter++]=mesh->vertexes[face[1]];
					storage.vertexes[counter++]=mesh->vertexes[face[2]];
					storage.vertexes[counter++]=mesh->vertexes[face[3]];
					springl.id=springls.size();
					storage.particles[pcounter]=springl.computeCentroid();
					openvdb::Vec3s norm=springl.computeNormal();
					storage.particleNormals[pcounter]=norm;
					storage.normals[counter-1]=norm;
					storage.normals[counter-2]=norm;
					storage.normals[counter-3]=norm;
					storage.normals[counter-4]=norm;
					springl.particle=&(storage.particles[pcounter]);
					springl.normal=&(storage.particleNormals[pcounter]);
					springls.push_back(springl);
				} else {
					storage.meshType=Mesh::PrimitiveType::TRIANGLES;
					storage.faces.push_back(openvdb::Vec4I(counter,counter+1,counter+2,openvdb::util::INVALID_IDX));
					Springl<Description,3> springl(&storage.vertexes[counter]);
					storage.vertexes[counter++]=mesh->vertexes[face[0]];
					storage.vertexes[counter++]=mesh->vertexes[face[1]];
					storage.vertexes[counter++]=mesh->vertexes[face[2]];
					springl.id=springls.size();
					openvdb::Vec3s norm=springl.computeNormal();
					storage.particleNormals[pcounter]=norm;
					storage.normals[counter-1]=norm;
					storage.normals[counter-2]=norm;
					storage.normals[counter-3]=norm;
					storage.normals[pcounter]=storage.particleNormals[pcounter]=springl.computeNormal();
					springl.particle=&(storage.particles[pcounter]);
					springl.normal=&(storage.particleNormals[pcounter]);
					springls.push_back(springl);
				}
				pcounter++;
			}
			storage.updateBBox();
		}
		inline void draw(bool colorEnabled=false){
			storage.draw(colorEnabled);
		}
		inline void updateGL(){
			storage.updateGL();
		}
		virtual ~Constellation(){}
		inline size_t size() const {return springls.size();}
		SpringlBase<Description>& operator[](size_t idx)
	    {
	    	return springls[idx];
	    }
		const SpringlBase<Description>& operator[](size_t idx) const
	    {
	    	return springls[idx];
	    }
};

template<typename Description> class SpringlRange
{
	typedef SpringlBase<Description> SpringlType;
	typedef Constellation<Description> ConstellationType;
public:
    class Iterator
    {
    public:
        Iterator(const SpringlRange& range, size_t pos): mRange(range), mPos(pos)
        {
            assert(this->isValid());
        }
        Iterator& operator=(const Iterator& other)
        {
            mRange = other.mRange; mPos = other.mPos; return *this;
        }
        /// Advance to the next leaf node.
        Iterator& operator++() { ++mPos; return *this; }
        /// Return a reference to the leaf node to which this iterator is pointing.
        SpringlType& operator*() const { return mRange.mConstellation[mPos]; }
        /// Return a pointer to the leaf node to which this iterator is pointing.
        SpringlType* operator->() const { return &(this->operator*()); }

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

    private:
        const SpringlRange& mRange;
        size_t mPos;
    };// end Iterator

    SpringlRange(size_t begin, size_t end, const ConstellationType& constellation, size_t grainSize=1):
        mEnd(end), mBegin(begin), mGrainSize(grainSize), mConstellation(constellation) {}

    SpringlRange(ConstellationType& constellation, size_t grainSize=1):
        mEnd(constellation.size()), mBegin(0), mGrainSize(grainSize), mConstellation(constellation) {}

    Iterator begin() const {return Iterator(*this, mBegin);}

    Iterator end() const {return Iterator(*this, mEnd);}

    size_t size() const { return mEnd - mBegin; }

    size_t grainsize() const { return mGrainSize; }

    const ConstellationType& constellation() const { return constellation; }

    bool empty() const {return !(mBegin < mEnd);}

    bool is_divisible() const {return mGrainSize < this->size();}

    SpringlRange(SpringlRange& r, tbb::split):
        mEnd(r.mEnd), mBegin(doSplit(r)), mGrainSize(r.mGrainSize),
          mConstellation(r.mConstellation) {}
    ConstellationType& mConstellation;
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

	template<typename Description,typename OperatorT,typename InterruptT = openvdb::util::NullInterrupter>
	class ConstellationOperator
	{
		typedef SpringlBase<Description> SpringlType;
	public:
		SpringlGrid& mGrid;
		ConstellationOperator(SpringlGrid& grid,InterruptT* _interrupt):mGrid(grid),mInterrupt(_interrupt){
		}

		virtual ~ConstellationOperator() {}
		void process(bool threaded = true)
		{
			if (mInterrupt) mInterrupt->start("Processing springls");
			OperatorT::init(mGrid);
			SpringlRange<Description> range(*mGrid.constellation);
			if (threaded) {
				tbb::parallel_for(range, *this);
			} else {
				(*this)(range);
			}

			if (mInterrupt) mInterrupt->end();
		}

		/// @note Never call this public method directly - it is called by
		/// TBB threads only!
		void operator()(const SpringlRange<Description>& range) const
		{
			if (openvdb::util::wasInterrupted(mInterrupt)) tbb::task::self().cancel_group_execution();
			for (typename SpringlRange<Description>::Iterator springl=range.begin(); springl; ++springl) {
				OperatorT::result(*springl,mGrid);
			}
		}

	protected:
		InterruptT*         mInterrupt;

	};
	template<typename Description>
	struct NearestNeighborOperation
	{
	private:

	public:
		static void init(SpringlGrid& mGrid){
			NearestNeighborMap& map=mGrid.nearestNeighbors;
			map.clear();
			map.resize(mGrid.constellation->size());
		}
	    static void result(const SpringlBase<Description>& springl,SpringlGrid& mGrid) {
	    	const int width=2;
	    	NearestNeighborMap& map=mGrid.nearestNeighbors;
	    	openvdb::math::DenseStencil<openvdb::Int32Grid>  stencil=openvdb::math::DenseStencil<openvdb::Int32Grid>(*mGrid.springlIndexGrid, width);
	        stencil.moveTo(openvdb::Coord(
	        		static_cast<openvdb::Int32>(std::floor((*springl.particle)[0]+0.5f)),
	        		static_cast<openvdb::Int32>(std::floor((*springl.particle)[1]+0.5f)),
	        		static_cast<openvdb::Int32>(std::floor((*springl.particle)[2]+0.5f))));
	        int sz=stencil.size();
	        if(sz==0)return;
	        std::vector<openvdb::Index32> stencilCopy(sz);

	        for(int i=0;i<sz;i++){
	        	stencilCopy[i]=stencil.getValue(i);
	        }
	        std::sort(stencilCopy.begin(),stencilCopy.end());
	        openvdb::Index32 last=stencilCopy[0];
	        for(int i=1;i<sz;i++){
	        	if(last!=stencilCopy[i]){
	        		if(last!=springl.id)map[springl.id].push_back(last);
	        		last=stencilCopy[i];
	        	}
	        }
	    }
	};
	// end of ConstellationOperator class
	/// @brief Compute the gradient of a scalar grid.
	template<
	    typename Description,
	    typename InterruptT = openvdb::util::NullInterrupter>
	class NearestNeighbors
	{
	public:
	    typedef Constellation<Description> ConstellationType;
	    NearestNeighbors(
	    			SpringlGrid& grid,
	    			InterruptT* interrupt = NULL):mGrid(grid),mInterrupt(interrupt)
	    {
	    }
	    void process(bool threaded = true)
	    {
        	typedef NearestNeighborOperation<Description> OpT;
	    	ConstellationOperator<Description,OpT,InterruptT> op(mGrid,mInterrupt);
        	op.process(threaded);
	    }
	    SpringlGrid& mGrid;
	    InterruptT*          mInterrupt;
	}; // end of Gradient class

}
#endif /* CONSTELLATION_H_ */
