///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2012-2013 DreamWorks Animation LLC
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of DreamWorks Animation nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////

#include "EnrightSpringls.h"
#include <iostream>
#include <string>
#include <vector>
#include <exception>
#include <stdlib.h>

#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/Filter.h>
#include <openvdb/Types.h>
#include <openvdb/openvdb.h>
#undef OPENVDB_REQUIRE_VERSION_NAME

#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetAdvect.h>
#include <openvdb/tools/LevelSetMeasure.h>
#include <openvdb/tools/LevelSetMorph.h>
#include <openvdb/tools/Morphology.h>
#include <openvdb/tools/PointAdvect.h>
#include <openvdb/tools/PointScatter.h>
#include <openvdb/tools/ValueTransformer.h>
#include <openvdb/tools/VectorTransformer.h>
#include <openvdb/util/Util.h>
#include <openvdb/math/Stats.h>
#include <boost/filesystem.hpp>
#include "Util.h"
#include <tbb/mutex.h>

#ifdef DWA_OPENVDB
#include <logging_base/logging.h>
#include <usagetrack.h>

#endif

using namespace openvdb;
tbb::mutex sLock;
int main(int argc, char *argv[]) {
	int status = EXIT_SUCCESS;

	try {
		if (argc > 2) {
			if (std::string(argv[1]) == "-playback") {

				std::string fileName(argv[2]);
				std::cout<<"Playback "<<argv[2]<<std::endl;
				openvdb::initialize();
				tbb::mutex::scoped_lock(sLock);
				OPENVDB_START_THREADSAFE_STATIC_WRITE
				imagesci::EnrightSpringls* viewer =
						imagesci::EnrightSpringls::GetInstance();
				OPENVDB_FINISH_THREADSAFE_STATIC_WRITE
				if(viewer->openRecording(fileName)){
					if(argc>3){
						std::cout<<"Frame index "<<atoi(argv[3])<<std::endl;
						viewer->setFrameIndex(atoi(argv[3]));
					}
					viewer->init(1200, 800);
				}

			} else if (std::string(argv[1]) == "-simulate") {
				std::string fileName(argv[2]);
				openvdb::initialize();
				tbb::mutex::scoped_lock(sLock);
				OPENVDB_START_THREADSAFE_STATIC_WRITE
				imagesci::EnrightSpringls* viewer =
						imagesci::EnrightSpringls::GetInstance();
				OPENVDB_FINISH_THREADSAFE_STATIC_WRITE
				std::string ext = boost::filesystem::extension(
						boost::filesystem::path(fileName));
				if (ext == std::string(".ply")) {
					viewer->openMesh(fileName);
				} else if (ext == std::string(".vdb")) {
					viewer->openGrid(fileName);
				}
				viewer->start();
				viewer->init(1200, 800);
			}
		} else {
			openvdb::initialize();
			tbb::mutex::scoped_lock(sLock);
			OPENVDB_START_THREADSAFE_STATIC_WRITE
			imagesci::EnrightSpringls* viewer =
					imagesci::EnrightSpringls::GetInstance();
			const float radius = 0.15f;
			const openvdb::Vec3f center(0.35f, 0.35f, 0.35f);
			int dim = 256;
			if(argc>1){
				dim=atoi(argv[1]);
			}
			float voxelSize = 1 / (float) (dim - 1);
			FloatGrid::Ptr signedLevelSet =
					openvdb::tools::createLevelSetSphere<FloatGrid>(radius,
							center, voxelSize);
			viewer->openGrid(*signedLevelSet);
			viewer->start();
			viewer->init(1200, 800);
		}
	} catch (imagesci::Exception& e) {
		std::cout << e.what() << std::endl;
		status = EXIT_FAILURE;
	}
	return status;
}
