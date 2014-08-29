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
#include <openvdb/tools/VolumeToMesh.h>
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

int Springl::size() const {
	return ((mesh->faces[id][3] == openvdb::util::INVALID_IDX) ? 3 : 4);
}

float Springl::distanceToParticle(const openvdb::Vec3s& pt) {
	return ((particle()) - pt).length();
}

float Springl::distanceToParticleSqr(const openvdb::Vec3s& pt) {
	return ((particle()) - pt).lengthSqr();
}
float Springl::distanceToFace(const openvdb::Vec3s& pt) {
	return std::sqrt(distanceToFaceSqr(pt));
}

float Springl::distanceToFaceSqr(const openvdb::Vec3s& pt) {
	Vec3s closest;
	if (size() == 3) {
		return DistanceToTriangleSqr(pt, (*this)[0], (*this)[1], (*this)[2],
				&closest);
	} else {
		return DistanceToQuadSqr(pt, (*this)[0], (*this)[1], (*this)[2],
				(*this)[3], normal(), &closest);
	}
}
float Springl::distanceEdgeSqr(const openvdb::Vec3s& pt, int e) {
	return DistanceToEdgeSqr(pt, (*this)[e], (*this)[(e + 1) % size()]);
}
float Springl::distanceEdge(const openvdb::Vec3s& pt, int e) {
	return std::sqrt(
			DistanceToEdgeSqr(pt, (*this)[e], (*this)[(e + 1) % size()]));
}

openvdb::Vec3s Springl::computeCentroid() const {
	openvdb::Vec3s centroid = openvdb::Vec3s(0.0f, 0.0f, 0.0f);
	int K = size();
	for (int k = 0; k < K; k++) {
		centroid += (*this)[k];
	}
	centroid = (1.0 / K) * centroid;
	return centroid;
}
openvdb::Vec3s Springl::computeNormal(const float eps) const {
	openvdb::Vec3s norm(0.0f);
	int K = size();
	openvdb::Vec3s pt = particle();
	for (int k = 0; k < K; k++) {
		norm += ((*this)[k] - pt).cross((*this)[(k + 1) % K] - pt);
	}

	norm.normalize(eps);
	return norm;
}
float Springl::area() const {
	openvdb::Vec3s norm(0.0f);
	int K = size();
	for (int k = 0; k < K; k++) {
		norm += ((*this)[k] - (particle())).cross(
				(*this)[(k + 1) % K] - (particle()));
	}
	return 0.5f * norm.length();
}
void SpringLevelSet::draw(bool colorEnabled, bool wireframe, bool particles,
		bool particleNormals) {
	/*
	 if (isoSurface.get() != nullptr) {
	 glColor3f(0.8f,0.3f, 0.3f);
	 isoSurface->draw(colorEnabled, wireframe, particles,
	 particleNormals);
	 }
	 */
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
const float SpringLevelSet::FILL_DISTANCE = 0.25f;
const float SpringLevelSet::SHARPNESS = 5.0f;
const float SpringLevelSet::SPRING_CONSTANT = 0.3f;
const float SpringLevelSet::RELAX_TIMESTEP = 0.1f;
const float SpringLevelSet::MAX_ANGLE_TOLERANCE = M_PI;
const float SpringLevelSet::MIN_ANGLE_TOLERANCE = 20 * M_PI / 180.0f;
const float SpringLevelSet::MIN_AREA = 0.05f;
openvdb::Vec3s& SpringLevelSet::GetParticle(const openvdb::Index32 id) {
	return (constellation->springls[id].particle());
}
openvdb::Vec3s& SpringLevelSet::GetParticleNormal(const openvdb::Index32 id) {
	return (constellation->springls[id].normal());
}
openvdb::Vec3s& SpringLevelSet::GetSpringlVertex(const openvdb::Index32 id,
		const int i) {
	return constellation->springls[id][i];
}
openvdb::Vec3s& SpringLevelSet::GetSpringlVertex(const openvdb::Index32 id) {
	return constellation->storage.vertexes[id];
}
Springl& SpringLevelSet::GetSpringl(const openvdb::Index32 id) {
	return constellation->springls[id];
}
void AdvectVertexOperation::init(SpringLevelSet& mGrid) {

}
void AdvectVertexOperation::result(Springl& springl, SpringLevelSet& mGrid) {
	int K = springl.size();
	for (int k = 0; k < K; k++) {
		springl[k] = mGrid.vertexDisplacement[springl.offset + k];
	}
}
void RelaxOperation::init(SpringLevelSet& mGrid) {
	mGrid.vertexDisplacement.resize(mGrid.constellation->getNumVertexes());
}
void RelaxOperation::result(Springl& springl, SpringLevelSet& mGrid) {
	float w, len;
	Vec3s tanget;
	Vec3s dir;
	int K = springl.size();
	std::vector<Vec3s> vertexVelocity(K);
	std::vector<Vec3s> tangets(K);
	std::vector<float> springForce(K);
	std::vector<float> tangetLengths(K);
	Vec3s particlePt = springl.particle();
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

			Springl& nbr = mGrid.GetSpringl(ci.springlId);
			DistanceToEdgeSqr(start, nbr[ci.edgeId],
					nbr[(ci.edgeId + 1) % nbr.size()], &pt2);
			dir = (pt2 - start);
			len = dir.length();
			w = ((len - 2 * SpringLevelSet::PARTICLE_RADIUS)
					/ (SpringLevelSet::MAX_VEXT
							+ 2 * SpringLevelSet::PARTICLE_RADIUS));

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
	map.resize(mGrid.constellation->getNumVertexes(),
			std::list<SpringlNeighbor>());
}
void NearestNeighborOperation::result(Springl& springl, SpringLevelSet& mGrid) {
	const float D2 = SpringLevelSet::NEAREST_NEIGHBOR_RANGE
			* SpringLevelSet::NEAREST_NEIGHBOR_RANGE;

	openvdb::math::DenseStencil<openvdb::Int32Grid> stencil =
			openvdb::math::DenseStencil<openvdb::Int32Grid>(
					*mGrid.springlIndexGrid,
					ceil(SpringLevelSet::NEAREST_NEIGHBOR_RANGE));
	openvdb::Vec3s refPoint = springl.particle();

	stencil.moveTo(
			Coord(std::floor(refPoint[0] + 0.5f),
					std::floor(refPoint[1] + 0.5f),
					std::floor(refPoint[2] + 0.5f)));

	int sz = stencil.size();
	if (sz == 0)
		return;
	Index32 N = mGrid.constellation->getNumSpringls();
	std::vector<std::pair<float, openvdb::Index32>> stencilCopy;
	for (int i = 0; i < sz; i++) {
		openvdb::Index32 id = stencil.getValue(i);
		if (id >= N)
			continue;
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
			Springl& snbr = mGrid.GetSpringl(nbrId);
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
		if (bestNbr.springlId >= 0 && bestNbr.springlId < N)
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
			openvdb::math::Transform::createLinearTransform(1.0f);
	using namespace openvdb::tools;
	using namespace openvdb;
	MeshToVolume<FloatGrid> mtol(trans, GENERATE_PRIM_INDEX_GRID);
	mtol.convertToUnsignedDistanceField(constellation->storage.vertexes,
			constellation->storage.faces, float(LEVEL_SET_HALF_WIDTH) * 2);
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

	//WriteToRawFile(unsignedLevelSet,"/home/blake/tmp/unsignedLevelSet");
	//std::cout << "Update gradient ..." << std::endl;
	//updateGradient();
	std::cout << "Update nearest neighbors ..." << std::endl;
	updateNearestNeighbors();
	std::cout << "Relax springls ..." << std::endl;
	relax(10);
	std::cout << "Clean ..." << std::endl;
	clean();
	updateUnsignedLevelSet();
	//WriteToRawFile(unsignedLevelSet,"/home/blake/tmp/unsigned_constl");
	//WriteToRawFile(springlIndexGrid,"/home/blake/tmp/index");
	std::cout << "Fill ..." << std::endl;
	fill();

	std::cout << "done." << std::endl;
}
void SpringLevelSet::fill() {

	openvdb::tools::VolumeToMesh mesher(0.0f);
	mesher(*signedLevelSet);

	openvdb::tree::ValueAccessor<FloatGrid::TreeType> acc(
			signedLevelSet->tree());
	openvdb::math::GenericMap map(signedLevelSet->transform());

	openvdb::math::DenseStencil<openvdb::Int32Grid> stencil =
			openvdb::math::DenseStencil<openvdb::Int32Grid>(*springlIndexGrid,
					ceil(FILL_DISTANCE));

	Index64 N = mesher.pointListSize();
	openvdb::tools::PolygonPoolList& polygonPoolList = mesher.polygonPoolList();
	Vec3s p[4];
	Vec3s refPoint;

	Mesh& storage = constellation->storage;
	Index32 springlsCount = constellation->getNumSpringls();
	Index32 pcounter = constellation->getNumSpringls();
	Index32 counter = constellation->getNumVertexes();

	int added = 0;
	float levelSetValue;
	std::vector<openvdb::Index32> stencilCopy;
	Index32 last;
	const float D2 = FILL_DISTANCE * FILL_DISTANCE;
	std::list<Index64> addList;
	for (Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
		const openvdb::tools::PolygonPool& polygons = polygonPoolList[n];
		for (Index64 i = 0, I = polygons.numQuads(); i < I; ++i) {
			const openvdb::Vec4I& quad = polygons.quad(i);
			p[0] = mesher.pointList()[quad[3]];
			p[1] = mesher.pointList()[quad[2]];
			p[2] = mesher.pointList()[quad[1]];
			p[3] = mesher.pointList()[quad[0]];
			refPoint = 0.25f * (p[0] + p[1] + p[2] + p[3]);
			stencil.moveTo(
					Coord(std::floor(refPoint[0] + 0.5f),
							std::floor(refPoint[1] + 0.5f),
							std::floor(refPoint[2] + 0.5f)));
			int sz = stencil.size();
			levelSetValue = std::numeric_limits<float>::max();
			stencilCopy.clear();
			last = -1;
			for (unsigned int nn = 0; nn < sz; nn++) {
				openvdb::Index32 id = stencil.getValue(nn);
				if (id >= springlsCount)
					continue;
				stencilCopy.push_back(id);
			}
			sz = stencilCopy.size();
			sort(stencilCopy.begin(), stencilCopy.end());
			for (unsigned int nn = 0; nn < sz; nn++) {
				openvdb::Index32 id = stencilCopy[nn];
				if (last != id) {
					float d = constellation->springls[id].distanceToFaceSqr(
							refPoint);
					if (d < levelSetValue) {
						levelSetValue = d;
					}
				}
				last = id;
			}
			if (levelSetValue > D2) {
				added++;
				storage.quadIndexes.push_back(counter);
				storage.vertexes.push_back(p[0]);
				storage.quadIndexes.push_back(counter + 1);
				storage.vertexes.push_back(p[1]);
				storage.quadIndexes.push_back(counter + 2);
				storage.vertexes.push_back(p[2]);
				storage.quadIndexes.push_back(counter + 3);
				storage.vertexes.push_back(p[3]);

				Springl springl(&storage);
				springl.offset = counter;
				springl.id = constellation->springls.size();
				constellation->storage.faces.push_back(
						Vec4I(counter, counter + 1, counter + 2, counter + 3));
				storage.particles.push_back(springl.computeCentroid());
				openvdb::Vec3s norm = springl.computeNormal();
				storage.particleNormals.push_back(norm);
				storage.vertexNormals.push_back(norm);
				storage.vertexNormals.push_back(norm);
				storage.vertexNormals.push_back(norm);
				storage.vertexNormals.push_back(norm);
				constellation->springls.push_back(springl);
				pcounter++;
				counter += 4;
			}
		}
		for (Index64 i = 0, I = polygons.numTriangles(); i < I; ++i) {
			const openvdb::Vec3I& tri = polygons.triangle(i);
			p[0] = mesher.pointList()[tri[2]];
			p[1] = mesher.pointList()[tri[1]];
			p[2] = mesher.pointList()[tri[0]];
			refPoint = 0.25f * (p[0] + p[1] + p[2]);
			stencil.moveTo(
					Coord(std::floor(refPoint[0] + 0.5f),
							std::floor(refPoint[1] + 0.5f),
							std::floor(refPoint[2] + 0.5f)));
			int sz = stencil.size();
			levelSetValue = std::numeric_limits<float>::max();
			stencilCopy.clear();
			last = -1;
			for (unsigned int nn = 0; nn < sz; nn++) {
				openvdb::Index32 id = stencil.getValue(nn);
				if (id >= springlsCount)
					continue;
				stencilCopy.push_back(id);
			}
			sz = stencilCopy.size();
			sort(stencilCopy.begin(), stencilCopy.end());
			for (unsigned int nn = 0; nn < sz; nn++) {
				openvdb::Index32 id = stencilCopy[nn];
				if (last != id) {
					float d = constellation->springls[id].distanceToFaceSqr(
							refPoint);
					if (d < levelSetValue) {
						levelSetValue = d;
					}
				}
				last = id;
			}
			if (levelSetValue > D2) {
				added++;
				storage.triIndexes.push_back(counter);
				storage.vertexes.push_back(p[0]);
				storage.triIndexes.push_back(counter + 1);
				storage.vertexes.push_back(p[1]);
				storage.triIndexes.push_back(counter + 2);
				storage.vertexes.push_back(p[2]);
				Springl springl(&storage);
				springl.offset = counter;
				springl.id = constellation->springls.size();

				constellation->storage.faces.push_back(
						Vec4I(counter, counter + 1, counter + 2,
								openvdb::util::INVALID_IDX));
				storage.particles.push_back(springl.computeCentroid());
				openvdb::Vec3s norm = springl.computeNormal();

				storage.particleNormals.push_back(norm);
				storage.vertexNormals.push_back(norm);
				storage.vertexNormals.push_back(norm);
				storage.vertexNormals.push_back(norm);
				constellation->springls.push_back(springl);
				pcounter++;
				counter += 3;
			}
		}
	}
	std::cout << "Added " << addList.size() << std::endl;
}
Constellation::Constellation(Mesh* mesh) :
		storage() {
	size_t faceCount = mesh->faces.size();
	size_t counter = 0;
	size_t pcounter = 0;
	springls.clear();
	storage.faces.clear();
	storage.quadIndexes.clear();
	storage.triIndexes.clear();
	storage.vertexes.resize(mesh->quadIndexes.size() + mesh->triIndexes.size());
	storage.particles.resize(faceCount);
	storage.particleNormals.resize(faceCount);
	storage.vertexNormals.resize(storage.vertexes.size());
	for (openvdb::Vec4I face : mesh->faces) {
		Springl springl(&storage);
		springl.offset = counter;
		springl.id = springls.size();
		if (face[3] != openvdb::util::INVALID_IDX) {
			storage.faces.push_back(
					openvdb::Vec4I(counter, counter + 1, counter + 2,
							counter + 3));
			storage.quadIndexes.push_back(counter);
			storage.quadIndexes.push_back(counter + 1);
			storage.quadIndexes.push_back(counter + 2);
			storage.quadIndexes.push_back(counter + 3);
			storage.vertexes[counter++] = mesh->vertexes[face[0]];
			storage.vertexes[counter++] = mesh->vertexes[face[1]];
			storage.vertexes[counter++] = mesh->vertexes[face[2]];
			storage.vertexes[counter++] = mesh->vertexes[face[3]];
			storage.particles[pcounter] = springl.computeCentroid();
			openvdb::Vec3s norm = springl.computeNormal();
			storage.particleNormals[pcounter] = norm;
			storage.vertexNormals[counter - 1] = norm;
			storage.vertexNormals[counter - 2] = norm;
			storage.vertexNormals[counter - 3] = norm;
			storage.vertexNormals[counter - 4] = norm;
			springls.push_back(springl);
		} else {
			storage.faces.push_back(
					openvdb::Vec4I(counter, counter + 1, counter + 2,
							openvdb::util::INVALID_IDX));
			storage.triIndexes.push_back(counter);
			storage.triIndexes.push_back(counter + 1);
			storage.triIndexes.push_back(counter + 2);
			storage.vertexes[counter++] = mesh->vertexes[face[0]];
			storage.vertexes[counter++] = mesh->vertexes[face[1]];
			storage.vertexes[counter++] = mesh->vertexes[face[2]];
			storage.particles[pcounter] = springl.computeCentroid();
			openvdb::Vec3s norm = springl.computeNormal();
			storage.particleNormals[pcounter] = norm;
			storage.vertexNormals[counter - 1] = norm;
			storage.vertexNormals[counter - 2] = norm;
			storage.vertexNormals[counter - 3] = norm;

			springls.push_back(springl);
		}
		pcounter++;
	}
	storage.updateBBox();
}

void SpringLevelSet::clean() {
	openvdb::math::BoxStencil<openvdb::FloatGrid> stencil(*signedLevelSet);
	Vec3s pt, pt1, pt2, pt3;
	float maxAngle, minAngle, area;
	int K;
	std::vector<Index32> keepList;
	Index32 newVertexCount = 0;
	Index32 newSpringlCount = 0;
	int N = constellation->getNumSpringls();
	keepList.reserve(N);
	Index32 index = 0;
	for (Springl& springl : constellation->springls) {
		pt = springl.particle();
		stencil.moveTo(
				Coord(std::floor(pt[0]), std::floor(pt[1]), std::floor(pt[2])));
		float levelSetValue = stencil.interpolation(pt);
		K = springl.size();

		if (fabs(levelSetValue) < 1.25f * SpringLevelSet::MAX_VEXT) {
			maxAngle = 0;
			minAngle = M_2_PI;
			area = 0.0f;
			for (int i = 0; i < K; i++) {
				pt1 = springl[i];
				pt2 = springl[(i + 1) % K];
				pt3 = springl[(i + 2) % K];
				float ang = Angle(pt1, pt2, pt3);
				maxAngle = std::max(maxAngle, ang);
				minAngle = std::min(minAngle, ang);
			}
			area = springl.area();
			if (area > MIN_AREA && maxAngle <= MAX_ANGLE_TOLERANCE
					&& minAngle >= MIN_ANGLE_TOLERANCE) {
				keepList.push_back(springl.id);
				newSpringlCount++;
				newVertexCount += K;
			}
		}
		/*

		 if (index % 2 == 0) {
		 keepList.push_back(springl.id);
		 newSpringlCount++;
		 newVertexCount += K;
		 }
		 */
		index++;
	}
	if (newSpringlCount == N)
		return;
	std::cout << "Removed " << N - newSpringlCount << std::endl;
	Index32 springlOffset = 0;
	Index32 vertexOffset = 0;
	Index32 quadIndex = 0;
	Index32 triIndex = 0;
	for (int n : keepList) {
		Springl& rspringl = constellation->springls[n];
		Springl& springl = constellation->springls[springlOffset];
		K = rspringl.size();
		if (springlOffset != n) {
			constellation->storage.particles[springlOffset] =
					constellation->storage.particles[n];
			constellation->storage.particleNormals[springlOffset] =
					constellation->storage.particleNormals[n];
			springl.offset = vertexOffset;
			springl.id = springlOffset;
			Vec4I quad;
			quad[3] = openvdb::util::INVALID_IDX;
			for (int k = 0; k < K; k++) {
				constellation->storage.vertexes[vertexOffset + k] =
						constellation->storage.vertexes[rspringl.offset + k];
				constellation->storage.vertexNormals[vertexOffset + k] =
						constellation->storage.vertexNormals[rspringl.offset + k];
				quad[k] = vertexOffset + k;
			}
			if (K == 4) {
				for (int k = 0; k < K; k++) {
					constellation->storage.quadIndexes[quadIndex++] =
							vertexOffset + k;
				}
			} else if (K == 3) {
				for (int k = 0; k < K; k++) {
					constellation->storage.triIndexes[triIndex++] = vertexOffset
							+ k;
				}
			}
			constellation->storage.faces[springlOffset] = quad;
		} else {
			if (K == 4) {
				quadIndex+=K;
			} else if (K == 3) {
				triIndex+=K;
			}
		}
		vertexOffset += K;
		springlOffset++;
	}
	constellation->storage.triIndexes.erase(
			constellation->storage.triIndexes.begin() + triIndex,
			constellation->storage.triIndexes.end());

	constellation->storage.quadIndexes.erase(
			constellation->storage.quadIndexes.begin() + quadIndex,
			constellation->storage.quadIndexes.end());

	constellation->springls.erase(
			constellation->springls.begin() + springlOffset,
			constellation->springls.end());
	constellation->storage.particles.erase(
			constellation->storage.particles.begin() + springlOffset,
			constellation->storage.particles.end());
	constellation->storage.particleNormals.erase(
			constellation->storage.particleNormals.begin() + springlOffset,
			constellation->storage.particleNormals.end());
	constellation->storage.faces.erase(
			constellation->storage.faces.begin() + springlOffset,
			constellation->storage.faces.end());

	constellation->storage.vertexNormals.erase(
			constellation->storage.vertexNormals.begin() + vertexOffset,
			constellation->storage.vertexNormals.end());
	constellation->storage.vertexes.erase(
			constellation->storage.vertexes.begin() + vertexOffset,
			constellation->storage.vertexes.end());
	/*
	 constellation->storage.vertexes.shrink_to_fit();
	 constellation->storage.normals.shrink_to_fit();
	 constellation->storage.particles.shrink_to_fit();
	 constellation->storage.particleNormals.shrink_to_fit();
	 */
}
}

