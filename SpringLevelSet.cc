/*
 * SpringLevelSet.cc
 *
 *  Created on: Aug 26, 2014
 *      Author: blake
 */
#include "SpringLevelSet.h"
#include <openvdb/Grid.h>
#include <openvdb/util/Util.h>
#include <openvdb/math/Stencils.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/openvdb.h>
namespace imagesci {
using namespace openvdb;
using namespace openvdb::math;
using namespace openvdb::tools;
std::ostream& operator<<(std::ostream& ostr, const SpringlNeighbor& classname) {
	ostr << "{" << classname.springlId << "|"
			<< static_cast<int>(classname.edgeId) << ":" << std::setprecision(4)
			<< classname.distance << "}";
	return ostr;
}

int8_t SpringlBase::size() const {
	return K;
}

float SpringlBase::distance(const openvdb::Vec3s& pt) {
	return ((*particle) - pt).length();
}

float SpringlBase::distanceSqr(const openvdb::Vec3s& pt) {
	return ((*particle) - pt).lengthSqr();
}
float SpringlBase::distanceEdgeSqr(const openvdb::Vec3s& pt, int8_t e) {
	return DistanceToEdge(pt, vertexes[e], vertexes[(e + 1) % K]);
}
float SpringlBase::distanceEdge(const openvdb::Vec3s& pt, int8_t e) {
	return std::sqrt(DistanceToEdge(pt, vertexes[e], vertexes[(e + 1) % K]));
}
openvdb::Vec3s SpringlBase::computeCentroid() const {
	openvdb::Vec3s centroid = openvdb::Vec3s(0.0f, 0.0f, 0.0f);
	for (int k = 0; k < K; k++) {
		centroid += (*this)[k];
	}
	centroid = (1.0 / K) * centroid;
	return centroid;
}
openvdb::Vec3s SpringlBase::computeNormal(const float eps) const {
	openvdb::Vec3s norm(0.0f);
	for (int k = 0; k < K; k++) {
		norm += (vertexes[(k + 1) % K] - (*particle)).cross(
				vertexes[k] - (*particle));
	}
	norm.normalize(eps);
	return norm;
}
float SpringlBase::area() const {
	openvdb::Vec3s norm(0.0f);
	for (int k = 0; k < K; k++) {
		norm += (vertexes[(k + 1) % K] - (*particle)).cross(
				vertexes[k] - (*particle));
	}
	return 0.5f*norm.length();
}
void SpringLevelSet::draw(bool colorEnabled, bool wireframe, bool particles,
		bool particleNormals) {
	if (constellation.get() != nullptr) {
		glColor3f(0.3f, 0.3f, 0.8f);
		constellation->draw(colorEnabled, wireframe, particles,
				particleNormals);
	}
}
const float SpringLevelSet::NEAREST_NEIGHBOR_RANGE = 1.5f;
const float SpringLevelSet::PARTICLE_RADIUS = 0.05f;
const float SpringLevelSet::MAX_VEXT = 0.5f;
const int SpringLevelSet::MAX_NEAREST_NEIGHBORS = 8;
const float SpringLevelSet::SHARPNESS = 5.0f;
const float SpringLevelSet::SPRING_CONSTANT = 0.3f;
const float SpringLevelSet::RELAX_TIMESTEP = 0.1f;
const float SpringLevelSet::MAX_ANGLE_TOLERANCE = M_PI;
const float SpringLevelSet::MIN_ANGLE_TOLERANCE = 20 * M_PI / 180.0f;
const float SpringLevelSet::MIN_AREA = 0.05f;
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
void AdvectVertexOperation::init(SpringLevelSet& mGrid) {

}
void AdvectVertexOperation::result(SpringlBase& springl,
		SpringLevelSet& mGrid) {
	int K = springl.size();
	//std::cout<<"Displacement ";
	for (int k = 0; k < K; k++) {
		//std::cout<<mGrid.vertexDisplacement[springl.offset+k]<<" ";
		springl[k] = mGrid.vertexDisplacement[springl.offset + k];
	}
	//std::cout<<std::endl;
}
void RelaxOperation::init(SpringLevelSet& mGrid) {
	mGrid.vertexDisplacement.resize(mGrid.constellation->getNumVertexes());
}
void RelaxOperation::result(SpringlBase& springl, SpringLevelSet& mGrid) {
	float w, len;
	Vec3s tanget;
	Vec3s dir;
	int K = springl.size();
	std::vector<Vec3s> vertexVelocity(K);
	std::vector<Vec3s> tangets(K);
	std::vector<float> springForce(K);
	std::vector<float> tangetLengths(K);
	Vec3s particlePt = *springl.particle;
	Vec3s startVelocity = Vec3s(0);
	Vec3s resultantMoment = Vec3s(0);
	const float MAX_FORCE = 0.999f;
	Vec3s start;
	float dotProd;
	Vec3s pt2;
	for (int k = 0; k < K; k++) {
		std::list<SpringlNeighbor>& map = mGrid.GetNearestNeighbors(springl.id,
				k);
		start = springl[k];
		// edge from pivot to magnet
		tanget = (start - particlePt);
		tangetLengths[k] = tanget.length();
		if (tangetLengths[k] > 1E-6f) {
			tanget *= (1.0f / tangetLengths[k]);
		}
		tangets[k] = tanget;
		startVelocity = Vec3s(0);
		// Sum forces
		//unroll loop
		for (SpringlNeighbor ci : map) {
			//Closest point should be recomputed each time and does not need to be stored
			SpringlBase& nbr = mGrid.GetSpringl(ci.springlId);
			DistanceToEdge(start, nbr[ci.edgeId],
					nbr[(ci.edgeId + 1) % nbr.size()], &pt2);
			dir = (pt2 - start);
			len = dir.length();
			w = ((len - 2 * SpringLevelSet::PARTICLE_RADIUS)
					/ (SpringLevelSet::MAX_VEXT
							+ 2 * SpringLevelSet::PARTICLE_RADIUS));

			//std::cout<<ci.springlId<<"W "<<w<<" "<<len<<" "<<pt2<<" "<<start<<std::endl;

			w = atanh(MAX_FORCE * clamp(w, -1.0f, 1.0f));
			startVelocity += (w * dir);
		}
		len = std::max(1E-6f, startVelocity.length());
		float t = SpringLevelSet::SHARPNESS * len;
		vertexVelocity[k] = SpringLevelSet::RELAX_TIMESTEP * startVelocity
				* (t / len);
		springForce[k] = SpringLevelSet::RELAX_TIMESTEP
				* SpringLevelSet::SPRING_CONSTANT
				* (2 * SpringLevelSet::PARTICLE_RADIUS - tangetLengths[k]);
		resultantMoment += vertexVelocity[k].cross(tangets[k]);
	}
	//std::cout<<"moment "<<resultantMoment<<" Normal "<<*springl.normal<<std::endl;
	openvdb::math::Mat3<float> rot = CreateAxisAngle(resultantMoment,
			-resultantMoment.length());
	//std::cout<<"Rotation\n"<<rot<<std::endl;

	//std::cout<<springl.id<<springl.offset<<" ROTATION "<<resultantMoment<<" "<<springForce[0]<<" "<<vertexVelocity[0]<<std::endl;
	for (int k = 0; k < K; k++) {
		start = springl[k] - particlePt;
		dotProd = std::max(
				start.length() + vertexVelocity[k].dot(tangets[k])
						+ springForce[k], 0.001f);
		start = dotProd * tangets[k];

		//disable rotation
		start = rot * start;
		mGrid.vertexDisplacement[springl.offset + k] = start + particlePt;
	}
}
void NearestNeighborOperation::init(SpringLevelSet& mGrid) {
	NearestNeighborMap& map = mGrid.nearestNeighbors;
	map.clear();
	map.resize(mGrid.constellation->getNumVertexes());
}
void NearestNeighborOperation::result(SpringlBase& springl,
		SpringLevelSet& mGrid) {
	const float D2 = SpringLevelSet::NEAREST_NEIGHBOR_RANGE
			* SpringLevelSet::NEAREST_NEIGHBOR_RANGE;

	openvdb::math::DenseStencil<openvdb::Int32Grid> stencil =
			openvdb::math::DenseStencil<openvdb::Int32Grid>(
					*mGrid.springlIndexGrid,
					ceil(SpringLevelSet::NEAREST_NEIGHBOR_RANGE));
	openvdb::Vec3f refPoint = *(springl.particle);

	stencil.moveTo(Coord(
			std::floor(refPoint[0]+0.5f),
			std::floor(refPoint[1]+0.5f),
			std::floor(refPoint[2]+0.5f)));

	int sz = stencil.size();
	if (sz == 0)
		return;
	std::vector<std::pair<float, openvdb::Index32>> stencilCopy;
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
	sz = stencilCopy.size();

	openvdb::Index32 last = -1;
	SpringlNeighbor bestNbr;

	for (int k = 0; k < springl.size(); k++) {
		std::list<SpringlNeighbor>& mapList = mGrid.GetNearestNeighbors(
				springl.id, k);
		refPoint = springl[k];
		bestNbr = SpringlNeighbor(-1, -1, D2);
		last = -1;
		for (int i = 0; i < sz; i++) {
			openvdb::Index32 nbrId = stencilCopy[i].second;
			if (nbrId == last)
				continue;
			SpringlBase& snbr = mGrid.GetSpringl(nbrId);
			for (int8_t n = 0; n < snbr.size(); n++) {
				float d = snbr.distanceEdgeSqr(refPoint, n);
				if (d <= bestNbr.distance) {
					bestNbr.springlId = nbrId;
					bestNbr.distance = d;
					bestNbr.edgeId = n;
				}
			}
			last = nbrId;
		}
		if (bestNbr.springlId >= 0)
			mapList.push_back(bestNbr);
	}
}

void SpringLevelSet::updateNearestNeighbors(bool threaded) {
	using namespace openvdb;
	NearestNeighbors<openvdb::util::NullInterrupter> nn(*this);
	nn.process();
	/*
	 std::vector<Index32>& lines = constellation->storage.lines;
	 lines.clear();
	 Index32 fCount = 0;
	 for (Index32 i = 0; i < constellation->getNumSpringls(); i++) {
	 //std::cout << "P " << fCount << "={";
	 SpringlBase& springl = constellation->springls[i];
	 for (int k = 0; k < springl.size(); k++) {
	 for (SpringlNeighbor nbr : GetNearestNeighbors(i, k)) {
	 lines.push_back(i);
	 lines.push_back(nbr.springlId);
	 Vec3s nbrPt = GetParticle(nbr.springlId);
	 //std::cout << nbr << " ";
	 }
	 }
	 //std::cout << "}" << std::endl;
	 fCount++;
	 }
	 */
}
void SpringLevelSet::relax(int iters) {
	Relax<openvdb::util::NullInterrupter> relax(*this);
	AdvectVertex<openvdb::util::NullInterrupter> advect(*this);
	for (int iter = 0; iter < iters; iter++) {
		relax.process();
		advect.process();
	}
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
std::list<SpringlNeighbor>& SpringLevelSet::GetNearestNeighbors(
		openvdb::Index32 id, int8_t e) {
	return nearestNeighbors[constellation->springls[id].offset + e];
}
void SpringLevelSet::create(Mesh* mesh) {
	openvdb::math::Transform::Ptr trans =
			openvdb::math::Transform::createLinearTransform(1.0);
	std::cout << "Convert mesh to volume ..." << std::endl;
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
	std::cout << "Update unsigned level set ..." << std::endl;
	updateUnsignedLevelSet();
	std::cout << "Update gradient ..." << std::endl;
	updateGradient();
	std::cout << "Update nearest neighbors ..." << std::endl;
	updateNearestNeighbors();
	std::cout << "Relax springls ..." << std::endl;
	relax(10);
	std::cout<<"Clean ..."<<std::endl;
	clean();
	std::cout << "done." << std::endl;
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
			springl.offset = counter;
			storage.vertexes[counter++] = mesh->vertexes[face[0]];
			storage.vertexes[counter++] = mesh->vertexes[face[1]];
			storage.vertexes[counter++] = mesh->vertexes[face[2]];
			storage.vertexes[counter++] = mesh->vertexes[face[3]];
			springl.id = springls.size();
			storage.particles[pcounter] = springl.computeCentroid();
			springl.particle = &(storage.particles[pcounter]);
			openvdb::Vec3s norm = springl.computeNormal();
			storage.particleNormals[pcounter] = norm;
			storage.normals[counter - 1] = norm;
			storage.normals[counter - 2] = norm;
			storage.normals[counter - 3] = norm;
			storage.normals[counter - 4] = norm;
			springl.normal = &(storage.particleNormals[pcounter]);
			springls.push_back(springl);
		} else {
			storage.meshType = Mesh::PrimitiveType::TRIANGLES;
			storage.faces.push_back(
					openvdb::Vec4I(counter, counter + 1, counter + 2,
							openvdb::util::INVALID_IDX));
			Springl<3> springl(&storage.vertexes[counter]);
			springl.offset = counter;
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
float SpringLevelSet::angle(openvdb::Vec3s& v0, openvdb::Vec3s& v1,
		openvdb::Vec3s& v2) {
	Vec3s v = v0 - v2;
	Vec3s w = v1 - v2;
	float len1 = v.length();
	float len2 = w.length();
	if (len1 <= 1E-3f || len2 <= 1E-3f)
		return 0;
	return std::acos(v.dot(w) / (len1 * len2));
}
void SpringLevelSet::clean() {
	openvdb::math::BoxStencil<openvdb::FloatGrid> stencil(*signedLevelSet);
	Vec3s pt, pt1, pt2, pt3;
	float maxAngle, minAngle,area;
	int K;
	std::vector<Index32> keepList;
	Index32 newVertexCount=0;
	Index32 newSpringlCount=0;
	int N=constellation->getNumSpringls();
	keepList.reserve(N);
	for (SpringlBase& springl : constellation->springls) {
		pt = *(springl.particle);
		stencil.moveTo(Coord(
				std::floor(pt[0]),
				std::floor(pt[1]),
				std::floor(pt[2])));
		float levelSetValue =stencil.interpolation(pt);

		if (fabs(levelSetValue) < 1.25f * SpringLevelSet::MAX_VEXT) {
			maxAngle = 0;
			minAngle = M_2_PI;
			K=springl.size();
			area=0.0f;
			for (int i = 0; i < K; i++) {
				pt1 = springl[i];
				pt2 = springl[(i + 1) % K];
				pt3 = springl[(i + 2) % K];
				float ang = angle(pt1, pt3, pt2);
				maxAngle = std::max(maxAngle, ang);
				minAngle = std::min(minAngle, ang);
			}
			area=springl.area();
			if (area > MIN_AREA && maxAngle <= MAX_ANGLE_TOLERANCE
					&& minAngle >= MIN_ANGLE_TOLERANCE) {
				keepList.push_back(springl.id);
				newSpringlCount++;
				newVertexCount+=K;
			}
		}
	}
	if(newSpringlCount==N)return;
	std::cout<<"Removed "<<N-newSpringlCount<<std::endl;
	std::vector<Index32> mask(N,std::numeric_limits<Index32>::max());
	Index32 springlOffset=0;
	Index32 vertexOffset=0;
	for(int n:keepList){
		mask[n]=springlOffset;
		SpringlBase& rspringl=constellation->springls[n];
		SpringlBase& springl=constellation->springls[springlOffset];
		K=rspringl.size();
		if(springlOffset!=n){
			constellation->storage.particles[springlOffset]=constellation->storage.particles[n];
			constellation->storage.particleNormals[springlOffset]=constellation->storage.particleNormals[n];
			springl.particle=&constellation->storage.particles[springlOffset];
			springl.normal=&constellation->storage.normals[springlOffset];
			springl.offset=vertexOffset;
			springl.id=springlOffset;
			for(int k=0;k<K;k++){
				springl.vertexes[k]=constellation->storage.vertexes[vertexOffset+k]=constellation->storage.vertexes[rspringl.offset+k];
				constellation->storage.normals[vertexOffset+k]=constellation->storage.normals[rspringl.offset+k];
			}
		}
		vertexOffset+=K;
		springlOffset++;
	}

	constellation->springls.erase(constellation->springls.begin()+springlOffset,constellation->springls.end());
	constellation->storage.particles.erase(constellation->storage.particles.begin()+springlOffset,constellation->storage.particles.end());
	constellation->storage.particleNormals.erase(constellation->storage.particleNormals.begin()+springlOffset,constellation->storage.particleNormals.end());

	constellation->storage.normals.erase(constellation->storage.normals.begin()+vertexOffset,constellation->storage.normals.end());
	constellation->storage.vertexes.erase(constellation->storage.vertexes.begin()+vertexOffset,constellation->storage.vertexes.end());
}
}

