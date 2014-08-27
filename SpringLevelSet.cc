/*
 * SpringLevelSet.cc
 *
 *  Created on: Aug 26, 2014
 *      Author: blake
 */
#include "SpringLevelSet.h"
namespace imagesci {
std::ostream& operator<<(std::ostream& ostr, const SpringlNeighbor& classname)
{
    ostr << "{"<<classname.springlId<<"|"<<static_cast<int>(classname.edgeId)<<"}";
    return ostr;
}

int8_t SpringlBase::size() const {return K;}

float SpringlBase::distance(const openvdb::Vec3s& pt){
	return ((*particle)-pt).length();
}

float SpringlBase::distanceSqr(const openvdb::Vec3s& pt){
	return ((*particle)-pt).lengthSqr();
}
float SpringlBase::distanceEdgeSqr(const openvdb::Vec3s& pt,int8_t e){
		return DistanceToEdge(pt,vertexes[e],vertexes[(e+1)%K]);
}
float SpringlBase::distanceEdge(const openvdb::Vec3s& pt,int8_t e){
		return std::sqrt(DistanceToEdge(pt,vertexes[e],vertexes[(e+1)%K]));
}
openvdb::Vec3s SpringlBase::computeCentroid() const{
	openvdb::Vec3s centroid=openvdb::Vec3s(0.0f,0.0f,0.0f);
	for(int k=0;k<K;k++){
		centroid+=(*this)[k];
	}
	centroid=(1.0/K)*centroid;
	return centroid;
}
openvdb::Vec3s SpringlBase::computeNormal(const float eps) const{
	openvdb::Vec3s norm;
	norm=(vertexes[2]-vertexes[0]).cross(vertexes[1]-vertexes[0]);
	norm.normalize(eps);
	return norm;
}
void SpringLevelSet::draw(bool colorEnabled) {
	/*
	 if(isoSurface.get()!=nullptr){
	 glColor3f(0.8f,0.3f,0.3f);
	 isoSurface->draw(colorEnabled);
	 }
	 */
	if (constellation.get() != nullptr) {
		glColor3f(0.3f, 0.3f, 0.8f);
		constellation->draw(colorEnabled);
	}
}
const float SpringLevelSet::NEAREST_NEIGHBOR_RANGE = 1.5f;
const int SpringLevelSet::MAX_NEAREST_NEIGHBORS = 8;

openvdb::Vec3s& SpringLevelSet::GetParticle(const openvdb::Index32 id) {
	return *(constellation->springls[id].particle);
}
openvdb::Vec3s& SpringLevelSet::GetParticleNormal(const openvdb::Index32 id) {
	return *(constellation->springls[id].normal);
}
openvdb::Vec3s& SpringLevelSet::GetSpringlVertex(const openvdb::Index32 id,
		const int i) {
	return constellation->springls[id][i];
}
openvdb::Vec3s& SpringLevelSet::GetSpringlVertex(const openvdb::Index32 id) {
	return constellation->storage.vertexes[id];
}
SpringlBase& SpringLevelSet::GetSpringl(const openvdb::Index32 id) {
	return constellation->springls[id];
}

void RelaxOperation::init(SpringLevelSet& mGrid) {
}
void RelaxOperation::result(const SpringlBase& springl,
		SpringLevelSet& mGrid) {

}
void NearestNeighborOperation::init(SpringLevelSet& mGrid) {
	NearestNeighborMap& map = mGrid.nearestNeighbors;
	map.clear();
	map.resize(mGrid.constellation->getNumVertexes());
}
void NearestNeighborOperation::result(const SpringlBase& springl,
		SpringLevelSet& mGrid) {

	openvdb::math::DenseStencil<openvdb::Int32Grid> stencil =
			openvdb::math::DenseStencil<openvdb::Int32Grid>(
					*mGrid.springlIndexGrid,
					ceil(SpringLevelSet::NEAREST_NEIGHBOR_RANGE));
	openvdb::Vec3f refPoint = *(springl.particle);
	stencil.moveTo(
			openvdb::Coord(
					static_cast<openvdb::Int32>(std::floor(refPoint[0] + 0.5f)),
					static_cast<openvdb::Int32>(std::floor(refPoint[1] + 0.5f)),
					static_cast<openvdb::Int32>(std::floor(refPoint[2] + 0.5f))));

	int sz = stencil.size();
	if (sz == 0)
		return;
	std::vector<std::pair<float, openvdb::Index32>> stencilCopy;
	float D2 = SpringLevelSet::NEAREST_NEIGHBOR_RANGE
			* SpringLevelSet::NEAREST_NEIGHBOR_RANGE;
	for (int i = 0; i < sz; i++) {
		openvdb::Index32 id = stencil.getValue(i);
		openvdb::Vec3s nbr = mGrid.GetParticle(id);
		float d = (refPoint - nbr).lengthSqr();
		if (id != springl.id) {
			stencilCopy.push_back(std::pair<float, openvdb::Index32>(d, id));
		}
	}
	if (stencilCopy.size() == 0)
		return;
	std::sort(stencilCopy.begin(), stencilCopy.end());
	openvdb::Index32 last = stencilCopy[0].second;

	last = -1;
	sz = stencilCopy.size();
	float smallest;
	int8_t smallestId;
	for (int i = 0; i < sz; i++) {
		openvdb::Index32 nbrId = stencilCopy[i].second;
		if (last != nbrId) {
			for(int k=0;k<springl.size();k++){
				std::list<SpringlNeighbor>& mapList = mGrid.GetNearestNeighbors(springl.id,k);
				refPoint=springl[k];
				SpringlBase& snbr = mGrid.GetSpringl(nbrId);
				smallest=D2;
				smallestId=-1;
				for (int8_t n = 0; n < snbr.size(); n++) {
					float d=snbr.distanceEdgeSqr(refPoint, n);
					if (d <= smallest) {
						smallest=d;
						smallestId=n;
					}
				}
				if(smallestId>=0)mapList.push_back(SpringlNeighbor(nbrId, smallestId));
			}
			last = stencilCopy[i].second;
		}
	}
}

void SpringLevelSet::updateNearestNeighbors(bool threaded) {
	using namespace openvdb;
	NearestNeighbors<openvdb::util::NullInterrupter> nn(*this);
	nn.process();
	std::vector<Index32>& lines = constellation->storage.lines;
	lines.clear();
	Index32 fCount = 0;
	for (Index32 i=0;i<constellation->getNumSpringls();i++) {
		std::cout << "P " << fCount << "={";
		SpringlBase& springl=constellation->springls[i];
		for(int k=0;k<springl.size();k++){
			for (SpringlNeighbor nbr : GetNearestNeighbors(i,k)) {
				lines.push_back(i);
				lines.push_back(nbr.springlId);
				Vec3s nbrPt = GetParticle(nbr.springlId);
				std::cout << nbr << " ";
			}
		}
		std::cout << "}" << std::endl;
		fCount++;
	}
	std::cout << "LINES " << lines.size() << std::endl;
}

void SpringLevelSet::updateUnsignedLevelSet() {
	openvdb::math::Transform::Ptr trans =
			openvdb::math::Transform::createLinearTransform();
	using namespace openvdb::tools;
	using namespace openvdb;
	MeshToVolume<FloatGrid> mtol(trans, GENERATE_PRIM_INDEX_GRID);
	mtol.convertToUnsignedDistanceField(isoSurface->vertexes, isoSurface->faces,
			float(LEVEL_SET_HALF_WIDTH) * 2);
	unsignedLevelSet = mtol.distGridPtr();
	unsignedLevelSet->setBackground(float(LEVEL_SET_HALF_WIDTH) * 2);
	springlIndexGrid = mtol.indexGridPtr();
}
void SpringLevelSet::updateGradient() {
	//gradient = openvdb::tools::gradient(*unsignedLevelSet);
	gradient = advectionForce(*unsignedLevelSet);

}
std::list<SpringlNeighbor>& SpringLevelSet::GetNearestNeighbors(openvdb::Index32 id,int8_t e){
		return nearestNeighbors[constellation->springls[id].offset+e];
}
void SpringLevelSet::create(Mesh* mesh) {
	openvdb::math::Transform::Ptr trans =
			openvdb::math::Transform::createLinearTransform(1.0);
	openvdb::tools::MeshToVolume<openvdb::FloatGrid> mtol(trans,
			openvdb::tools::GENERATE_PRIM_INDEX_GRID);
	mtol.convertToLevelSet(mesh->vertexes, mesh->faces);
	signedLevelSet = mtol.distGridPtr();
	springlIndexGrid = mtol.indexGridPtr();
	Mesh* m = new Mesh();
	m->create(signedLevelSet);
	isoSurface = std::unique_ptr<Mesh>(m);
	Constellation* c = new Constellation(m);
	constellation = boost::shared_ptr<Constellation>(c);
	updateUnsignedLevelSet();
	updateGradient();
	updateNearestNeighbors();
}
Constellation::Constellation(Mesh* mesh) :
		storage() {
	size_t faceCount = mesh->faces.size();
	size_t counter = 0;
	size_t pcounter = 0;
	springls.reserve(faceCount);
	storage.faces.reserve(faceCount);
	storage.vertexes.resize(mesh->indexes.size());
	storage.particles.resize(faceCount);
	storage.particleNormals.resize(faceCount);
	storage.normals.resize(mesh->indexes.size());
	for (openvdb::Vec4I face : mesh->faces) {
		if (face[3] != openvdb::util::INVALID_IDX) {
			storage.meshType = Mesh::PrimitiveType::QUADS;
			storage.faces.push_back(
					openvdb::Vec4I(counter, counter + 1, counter + 2,
							counter + 3));
			Springl<4> springl(&(storage.vertexes[counter]));
			springl.offset=counter;
			storage.vertexes[counter++] = mesh->vertexes[face[0]];
			storage.vertexes[counter++] = mesh->vertexes[face[1]];
			storage.vertexes[counter++] = mesh->vertexes[face[2]];
			storage.vertexes[counter++] = mesh->vertexes[face[3]];
			springl.id = springls.size();
			storage.particles[pcounter] = springl.computeCentroid();
			openvdb::Vec3s norm = springl.computeNormal();
			storage.particleNormals[pcounter] = norm;
			storage.normals[counter - 1] = norm;
			storage.normals[counter - 2] = norm;
			storage.normals[counter - 3] = norm;
			storage.normals[counter - 4] = norm;
			springl.particle = &(storage.particles[pcounter]);
			springl.normal = &(storage.particleNormals[pcounter]);
			springls.push_back(springl);
		} else {
			storage.meshType = Mesh::PrimitiveType::TRIANGLES;
			storage.faces.push_back(
					openvdb::Vec4I(counter, counter + 1, counter + 2,
							openvdb::util::INVALID_IDX));
			Springl<3> springl(&storage.vertexes[counter]);
			springl.offset=counter;
			storage.vertexes[counter++] = mesh->vertexes[face[0]];
			storage.vertexes[counter++] = mesh->vertexes[face[1]];
			storage.vertexes[counter++] = mesh->vertexes[face[2]];
			springl.id = springls.size();
			openvdb::Vec3s norm = springl.computeNormal();
			storage.particleNormals[pcounter] = norm;
			storage.normals[counter - 1] = norm;
			storage.normals[counter - 2] = norm;
			storage.normals[counter - 3] = norm;
			storage.normals[pcounter] = storage.particleNormals[pcounter] =
					springl.computeNormal();
			springl.particle = &(storage.particles[pcounter]);
			springl.normal = &(storage.particleNormals[pcounter]);
			springls.push_back(springl);
		}
		pcounter++;
	}
	storage.updateBBox();
}
}

