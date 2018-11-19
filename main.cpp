#include <iostream>

#include "windfarm.h"
#include "windfarmMover.h"
#include "multilevelqualitythreshold.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/chrono.hpp>


int main(int argc, char **argv) {
	int nS = 42;
	auto clock = boost::chrono::process_user_cpu_clock();
	auto beginning = clock.now();
    try {
        boost::program_options::options_description desc("Allowed Options");
        desc.add_options()
        ("help", "produce help message" )
        ("mover", "clustering with mover algorithm")
        ("singleMove", "mover clustering but if no improvement only move single nodes, children are adopted by grandparent")
        ("adoption", "mover clustering, but if no improvements, move single nodes and adopt some of the children of the new parent by moved node")
        ("mqt", "multilevel quality threshold as in the paper \"Optimal Wind Farm Collector System Topology Design Considering Total Trenching Length\" " )
        ("mqtCost", "multilevel quality Treshold but Considering the cable costs not just the distance")
		("mst", "computes an MST, based on the distances")
		("nS", boost::program_options::value<int>(&nS)->default_value(42), "Substation capacity")
		("initialize", "initialize test")
		("stats", "gives some stats about the graph")
        ("input-file", boost::program_options::value<std::string>(), "input file");

        boost::program_options::positional_options_description p;
        p.add("input-file", -1);

        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        boost::program_options::notify(vm);

        if (vm.count("help")) {
            std::cout << desc << "\n";
            return 0;
        }

        if (!vm.count("input-file")) {
            std::cout << "please specify an input file" << std::endl;
            return 2;
        }
        boost::filesystem::path filename {vm["input-file"].as<std::string>()};
        boost::filesystem::path outputpath = filename.parent_path() / filename.stem();
        boost::filesystem::create_directory(outputpath);

        Windfarm wf(filename.native(), nS);
				
        if (vm.count("mover")) {
            WindfarmMover mover(wf);
			//ClusterFunc func = std::bind(&WindfarmMover::computeClustering, &mover);
            //mover.findBest(func);
			mover.computeClustering();
            wf.cableToSVG((outputpath /  "mover.svg").native());

        }
        if (vm.count("singleMove")) {
            WindfarmMover mover(wf);
			ClusterFunc func = std::bind(&WindfarmMover::singleMove, &mover);
			//mover.findBest(func);
			mover.singleMove();
			wf.cableToSVG((outputpath / "singleMove.svg").native());
			
        }
        if (vm.count("adoption")) {
            WindfarmMover mover(wf);
			ClusterFunc func = std::bind(&WindfarmMover::adoptionMove, &mover);
// 			mover.findBest(func);
			mover.adoptionMove();
			wf.cableToSVG((outputpath / "adoption.svg").native());
			
        }
        if (vm.count("mqt")) {
            MultiLevelQualityThreshold mqt(wf);
            mqt.findBestInitialThresholdDistance();
             wf.cableToSVG((outputpath / "mqt.svg").native());
        }
        if (vm.count("mqtCost")) {
            MultiLevelQualityThreshold mqt(wf);
            mqt.findBestInitialThresholdCost();
             wf.cableToSVG((outputpath / "mqtCost.svg").native());
        }
        if (vm.count("initialize")) {
			auto result = wf.initializeCableLayoutDistance();
			wf.cableLayout_ = std::move(result.first);
			wf.cableLayoutGA_ = std::move(result.second);
			wf.cableToSVG((outputpath / "initializedDistance.svg").native());
		}
        if (vm.count("mst")){
 			auto MST = wf.buildMST();
			wf.cableLayout_ = std::move(MST.first);
			wf.cableLayoutGA_ = std::move(MST.second);
 			wf.cableToSVG((outputpath / "mst.svg").native());
		}
		if (vm.count("stats")){
			std::cout << "Anzahl Turbinen, Anzahl Trafostationen, Seitenverhältnis" << std::endl;
			std::cout << wf.turbines_.size() << std::endl;
			std::cout << wf.substations_.size() << std::endl;
			std::cout << wf.getAspectRatio() << std::endl;
		}
		
        std::cout << wf.realCost() << std::endl;
		auto endTime = clock.now();
		auto duration = endTime -beginning;
		std::cout << boost::chrono::round<boost::chrono::milliseconds>(duration).count() << std::endl;
		return 0;
		
    }
    catch(std::invalid_argument& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 2;
    }
    catch(std::domain_error& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    return 0;

}
