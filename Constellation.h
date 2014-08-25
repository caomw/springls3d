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
	virtual ~SpringlGrid();
};
template<typename Description> class SpringlBase {
	public:
		openvdb::Index32 id;
		openvdb::Vec3s* particle;
		openvdb::Vec3s* normal;
		Description description;
		virtual size_t size();
		virtual openvdb::Vec3s& operator[](size_t idx);
		virtual const openvdb::Vec3s& operator[](size_t idx) const;
		virtual ~SpringlBase();
		virtual openvdb::Vec3s computeCentroid() const;
		virtual openvdb::Vec3s computeNormal(const float eps=1E-6f) const;

		SpringlBase():id(0),particle(NULL),normal(NULL){
		}
};
template<typename Description,size_t K> class Springl: public SpringlBase<Description> {
	public:
		std::array<openvdb::Vec3s*,K> vertexes;
		openvdb::Vec3s& operator[](size_t idx){return (*vertexes[idx]);}
		const openvdb::Vec3s& operator[](size_t idx) const {return *vertexes[idx];}
		inline size_t size() const {return K;}
		openvdb::Vec3s computeCentroid() const{
			openvdb::Vec3s centroid=openvdb::Vec3s(0.0f,0.0f,0.0f);
			for(openvdb::Vec3s v:vertexes){
				centroid+=v;
			}
			centroid=(1.0/size())*centroid;
			return centroid;
		}
		openvdb::Vec3s computeNormal(const float eps) const{
			openvdb::Vec3s norm;
			norm=(this[2]-this[0]).cross(this[1]-this[0]);
			norm.normalize(eps);
			return norm;
		}

		~Springl(){}
};

template<typename Description> using TriSpringl=Springl<Description,3> ;
template<typename Description> using QuadSpringl=Springl<Description,4> ;

template<typename Description> class Constellation {
protected:
		Mesh storage;
		std::vector<SpringlBase<Description>> springls;
public:
		Constellation(Mesh& mesh);
		inline void draw(bool colorEnabled=false){
			storage.draw(colorEnabled);
		}
		inline void updateGL(){
			storage.updateGL();
		}
		virtual ~Constellation();
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
		const SpringlGrid& mGrid;
		ConstellationOperator(const SpringlGrid& grid,InterruptT* _interrupt):mGrid(grid),mInterrupt(_interrupt){
		}

		virtual ~ConstellationOperator() {}
		void process(bool threaded = true)
		{
			if (mInterrupt) mInterrupt->start("Processing springls");
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
				OperatorT::result(*springl);
			}
		}

	protected:
		InterruptT*         mInterrupt;

	};
	template<typename Description>
	struct NearestNeighborOperation
	{
	private:
	    const openvdb::Int32Grid::Ptr& mSpringlGrid;
	    NearestNeighborMap& mNearestNeighbors;
	public:
	    static void result(const SpringlBase<Description>& springl) {
	    	//Do something
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
	    			const SpringlGrid& grid,
	    			InterruptT* interrupt = NULL):mGrid(grid),mInterrupt(interrupt)
	    {
	    }
	    void process(bool threaded = true)
	    {
        	typedef NearestNeighborOperation<Description> OpT;
	    	ConstellationOperator<Description,OpT,InterruptT> op(mGrid,mInterrupt);
        	op.process(threaded);
	    }
	    const SpringlGrid& mGrid;
	    InterruptT*          mInterrupt;
	}; // end of Gradient class

}
#endif /* CONSTELLATION_H_ */
