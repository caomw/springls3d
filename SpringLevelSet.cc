/*
 * TrackingGrid.cc
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#include "SpringLevelSet.h"
#include "ImageSciUtil.h"
#include "AdvectionForce.h"
#include <openvdb/tools/MeshToVolume.h>
#include <iterator>
using namespace openvdb;
using namespace openvdb::tools;
namespace imagesci {

const float SpringLevelSet::NEAREST_NEIGHBOR_RANGE=1.5f;
const int SpringLevelSet::MAX_NEAREST_NEIGHBORS=8;

openvdb::Vec3s& SpringLevelSet::GetParticle(openvdb::Index32 id){
	return *(constellation->springls[id].particle);
}
openvdb::Vec3s& SpringLevelSet::GetParticleNormal(openvdb::Index32 id){
	return *(constellation->springls[id].normal);
}
openvdb::Vec3s& SpringLevelSet::GetSpringlVertex(openvdb::Index32 id,int i){
	return constellation->springls[id][i];
}

bool SpringLevelSet::create(const Mesh& mesh,openvdb::math::Transform::Ptr& transform){
	MeshToVolume<FloatGrid> mtol(transform,GENERATE_PRIM_INDEX_GRID);
	mtol.convertToLevelSet(mesh.vertexes,mesh.faces);
	signedLevelSet=mtol.distGridPtr();
	springlIndexGrid=mtol.indexGridPtr();
    Mesh* m=new Mesh();
	m->create(signedLevelSet);
	isoSurface=std::unique_ptr<Mesh>(m);
	Constellation<Int32>* c=new Constellation<openvdb::Int32>(m);
	constellation=boost::shared_ptr<Constellation<openvdb::Int32>>(c);
	updateUnsignedLevelSet();
	updateGradient();
	updateNearestNeighbors();
	return true;
}
void SpringLevelSet::draw(bool colorEnabled){

	if(isoSurface.get()!=nullptr){
		glColor3f(0.8f,0.3f,0.3f);
		isoSurface->draw(colorEnabled);
	}
	if(constellation.get()!=nullptr){
		glColor3f(0.3f,0.3f,0.8f);
		constellation->draw(colorEnabled);
	}
}
void SpringLevelSet::updateNearestNeighbors(bool threaded){
	NearestNeighbors<openvdb::Int32> nn(*this);
	nn.process();
	std::vector<Index32>& lines=constellation->storage.lines;
	lines.clear();
	Index32 fCount=0;
	for(std::list<SpringlNeighbor>& nbrs:nearestNeighbors){
		//std::cout<<"P "<<fCount<<"={";
		Vec3s refPoint=GetParticle(fCount);
		for(SpringlNeighbor nbr:nbrs){
			lines.push_back(fCount);
			lines.push_back(nbr.springlId);
			Vec3s nbrPt=GetParticle(nbr.springlId);
			float d=(refPoint-nbrPt).lengthSqr();
			//std::cout<<"["<<nbr<<","<<d<<"] ";
		}
		//std::cout<<"}"<<std::endl;
		fCount++;
	}
	std::cout<<"LINES "<<lines.size()<<std::endl;
}

void SpringLevelSet::updateUnsignedLevelSet(){
	openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
	MeshToVolume<FloatGrid> mtol(trans,GENERATE_PRIM_INDEX_GRID);
	mtol.convertToUnsignedDistanceField(isoSurface->vertexes,isoSurface->faces,float(LEVEL_SET_HALF_WIDTH)*2);
	unsignedLevelSet=mtol.distGridPtr();
	unsignedLevelSet->setBackground(float(LEVEL_SET_HALF_WIDTH)*2);
	springlIndexGrid=mtol.indexGridPtr();
}
void SpringLevelSet::updateGradient(){
	//gradient = openvdb::tools::gradient(*unsignedLevelSet);
	gradient = advectionForce(*unsignedLevelSet);

}
SpringLevelSet::SpringLevelSet() {
}
SpringLevelSet::~SpringLevelSet() {
}
template<typename Description> void NearestNeighborOperation<Description>::init(SpringLevelSet& mGrid){
	NearestNeighborMap& map=mGrid.nearestNeighbors;
	map.clear();
	map.resize(mGrid.constellation->size());
}
template<typename Description> void NearestNeighborOperation<Description>::result(const SpringlBase<Description>& springl,SpringLevelSet& mGrid) {
	std::list<SpringlNeighbor>& mapList=mGrid.GetNearestNeighbors(springl.id);
	openvdb::math::DenseStencil<openvdb::Int32Grid>  stencil=openvdb::math::DenseStencil<openvdb::Int32Grid>(*mGrid.springlIndexGrid,ceil(SpringLevelSet::NEAREST_NEIGHBOR_RANGE));
    openvdb::Vec3f refPoint=*(springl.particle);
	stencil.moveTo(openvdb::Coord(
    		static_cast<openvdb::Int32>(std::floor(refPoint[0]+0.5f)),
    		static_cast<openvdb::Int32>(std::floor(refPoint[1]+0.5f)),
    		static_cast<openvdb::Int32>(std::floor(refPoint[2]+0.5f))));

	int sz=stencil.size();
    if(sz==0)return;
    std::vector<std::pair<float,openvdb::Index32>> stencilCopy;
    float D2=SpringLevelSet::NEAREST_NEIGHBOR_RANGE*SpringLevelSet::NEAREST_NEIGHBOR_RANGE;
    for(int i=0;i<sz;i++){
    	openvdb::Index32 id=stencil.getValue(i);
		openvdb::Vec3s nbr=mGrid.GetParticle(id);
		float d=(refPoint-nbr).lengthSqr();
    	if(id!=springl.id&&d<D2){
    		stencilCopy.push_back(std::pair<float,openvdb::Index32>(d,id));
    	}
    }
    if(stencilCopy.size()==0)return;
    std::sort(stencilCopy.begin(),stencilCopy.end());
    openvdb::Index32 last=stencilCopy[0].second;
    mapList.clear();
    mapList.push_back(SpringlNeighbor(last,-1));
    sz=stencilCopy.size();
    for(int i=1;i<sz;i++){
    	if(last!=stencilCopy[i].second){
    		mapList.push_back(SpringlNeighbor(stencilCopy[i].second));
    		if(mapList.size()>=SpringLevelSet::MAX_NEAREST_NEIGHBORS)return;
    		last=stencilCopy[i].second;
    	}
    }
}
}

