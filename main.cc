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
#include <tbb/mutex.h>

#ifdef DWA_OPENVDB
#include <logging_base/logging.h>
#include <usagetrack.h>

#endif
#include "Util.h"
#include "SimulationVisualizer.h"
#include "SimulationComparisonVisualizer.h"
#include "EnrightSimulation.h"
#include "SimulationPlayback.h"
#include "ArmadilloTwist.h"
#include <iostream>
using namespace openvdb;
using namespace imagesci;
using namespace std;
int main(int argc, char *argv[]) {
	int status = EXIT_FAILURE;
	std::vector<std::string> args;
	for(int i=0;i<argc;i++){
		args.push_back(argv[i]);
		cout<<argv[i]<<" ";
	}

	cout<<endl;
	const int WIN_WIDTH=1280;
	const int WIN_HEIGHT=720;
	try {
		openvdb::initialize();
		for(int i=0;i<args.size();i++){
			if( args[i]== "-compare") {
				std::string dirName1=std::string(args[++i]);
				std::string dirName2=std::string(args[++i]);
				SimulationPlayback sim1(dirName1);
				SimulationPlayback sim2(dirName2);
				SimulationComparisonVisualizer::run(&sim1,&sim2,WIN_WIDTH,WIN_HEIGHT);
				status=EXIT_SUCCESS;
			} else if( args[i]== "-playback") {
				std::string dirName=std::string(args[++i]);
				SimulationPlayback sim(dirName);
				SimulationVisualizer::run(static_cast<Simulation*>(&sim),WIN_WIDTH,WIN_HEIGHT,dirName);
				status=EXIT_SUCCESS;
			} else if(args[i]== "-enright"){
				std::string dirName=std::string(argv[++i]);
				int dim = 256;
				MotionScheme scheme=DecodeMotionScheme(args[++i]);
				if(argc>3){
					dim=atoi(argv[++i]);
				}
				EnrightSimulation sim(dim,scheme);
				SimulationVisualizer::run(static_cast<Simulation*>(&sim),WIN_WIDTH,WIN_HEIGHT,dirName);
				status=EXIT_SUCCESS;
			} else if(args[i]=="-twist"){
				std::string dirName=std::string(argv[++i]);
				std::string sourceFileName="armadillo.ply";
				MotionScheme scheme=DecodeMotionScheme(args[++i]);
				if(argc>3){
					sourceFileName=std::string(argv[++i]);
				}
				ArmadilloTwist sim(sourceFileName,scheme);
				SimulationVisualizer::run(static_cast<Simulation*>(&sim),WIN_WIDTH,WIN_HEIGHT,dirName);
				status=EXIT_SUCCESS;
			}
		}
	} catch (imagesci::Exception& e) {
		cout << "ImageSci Error:: "<< e.what() << endl;
	} catch (openvdb::Exception& e) {
		cout << "OpenVDB Error:: "<< e.what() << endl;
	}
	if(status==EXIT_FAILURE){
		cout<<"Usage: "<<argv[0]<<" -playback <INPUT_DIRECTORY>"<<endl;
		cout<<"Usage: "<<argv[0]<<" -enright <OUTPUT_DIRECTORY> <implicit|semi-implicit|explicit> <INTEGER_GRID_SIZE>"<<endl;
		cout<<"Usage: "<<argv[0]<<" -twist <OUTPUT_DIRECTORY> <implicit|semi-implicit|explicit> <MESH_FILE>"<<endl;
	}
	return status;
}
