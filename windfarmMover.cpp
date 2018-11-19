/*
 * Copyright 2017 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "windfarmMover.h"

WindfarmMover::WindfarmMover(Windfarm& wf, int stop) :   wf_(wf), stop_(stop) {

    //Cluster initialisieren (Jeder Knoten sein eigener Cluster)
    cluster_ = initializeCluster();

    cost_total_ = wf_.costTotal(*wf_.cableLayout_, *wf_.cableLayoutGA_);
}

ogdf::SList< ogdf::SimpleCluster* > WindfarmMover::initializeCluster() const {
    auto cluster = ogdf::SList<ogdf::SimpleCluster*>();

    for(auto v: wf_.getGraph().nodes) {
        ogdf::SimpleCluster* clust = new ogdf::SimpleCluster;
        clust->setIndex(v->index());
        cluster.pushBack(clust);
    }

    return cluster;
}

double WindfarmMover::averageCIndex() {
    return wf_.costTotal(*wf_.cableLayout_, *wf_.cableLayoutGA_);
}

ClusterResult WindfarmMover::computeClustering() const {
	auto initialCableLayout = wf_.initializeCableLayout();
    auto cableLayoutNew = std::move(initialCableLayout.first);
    auto cableLayoutGANew = std::move(initialCableLayout.second);
 	wf_.cableToSVG("moverInitial.svg", *cableLayoutNew, *cableLayoutGANew);
    auto cluster = initializeCluster();
    double cost = wf_.costTotal(*cableLayoutNew, *cableLayoutGANew);
	//std::cout << "Kosten initial: " << cost << std::endl;
    double cOld = cost +1;
    int counter = 0;

    // main loop
    while((cOld > cost) && counter < stop_ ) {
		std::cout << "counter = " << counter << ", stop_ = " << stop_ << std::endl;
        cOld = cost;
		double cTemp = cost;
        std::shuffle(wf_.turbines_.begin(), wf_.turbines_.end(), std::default_random_engine(std::random_device()()));

        for(auto t: wf_.turbines_) {
            ogdf::SimpleCluster* clustT = graphalgos::getRootCluster(t->index(), cluster);

            ogdf::SimpleCluster* clustTemp = clustT;
            ogdf::node const parentNode = graphalgos::parent(t, *cableLayoutNew);

            auto Gtemp = std::make_unique<ogdf::Graph> (*cableLayoutNew);
            auto GATemp = std::make_unique<ogdf::GraphAttributes> (*Gtemp,
                          ogdf::GraphAttributes::nodeId |
                          ogdf::GraphAttributes::edgeIntWeight);

            //Fluss kopieren
            for(auto e: Gtemp->edges) {
                ogdf::node sourceOrig = searchNode(e->source()->index(), *cableLayoutNew);
                ogdf::node targetOrig = searchNode(e->target()->index(), *cableLayoutNew);
                ogdf::edge eOrig = cableLayoutNew->searchEdge(sourceOrig, targetOrig);
                GATemp->intWeight(e) = cableLayoutGANew->intWeight(eOrig);
            }
            
            //günstigste Kante zum Clustern suchen, falls sie existiert
            
            for(ogdf::adjEntry adj: t->adjEntries) {
                auto n = adj->theEdge()->opposite(t);

                auto flowNew = wf_.redirect(parentNode->index(), t->index() , n->index(), cost, *cableLayoutNew, *cableLayoutGANew);
                if (!flowNew) {
                    continue;
                }
				if (std::get<2>(*flowNew) <= cTemp) {
					cTemp = std::get<2>(*flowNew);
                    clustTemp = graphalgos::getRootCluster(n->index(), cluster);
                    GATemp = std::move(std::get<1>(*flowNew));
                    Gtemp = std::move(std::get<0>(*flowNew));
                }
            }
            if(clustTemp != clustT) {
                graphalgos::mergeClusters(*clustTemp, *clustT, cluster);
                cableLayoutNew = std::move(Gtemp);
                cableLayoutGANew = std::move(GATemp);
				cost = cTemp;
				int pos = 0;
				auto it = std::find(wf_.turbines_.begin(), wf_.turbines_.end(), t);
				if (it == wf_.turbines_.end())
				{
					// name not in vector
				} else
				{
					 pos = std::distance(wf_.turbines_.begin(), it);
				}
				std::cout<< "cost: " << cost << ", counter: " << counter << ", step: " << pos << "node: " << t << std::endl;
 				wf_.cableToSVG("mover-" + std::to_string(counter) + "step-" + std::to_string(pos) + "-node-" + std::to_string(t->index()) + ".svg", *cableLayoutNew, *cableLayoutGANew);
            }
        }
//  		wf_.cableToSVG("test-" + std::to_string(counter) + ".svg", *cableLayoutNew, *cableLayoutGANew);
        counter++;
    }

    return std::make_tuple(std::move(cableLayoutNew), std::move(cableLayoutGANew), cost, cluster);
}

ClusterResult WindfarmMover::singleMove() {
	auto initialCableLayout = wf_.initializeCableLayout();
    auto cableLayoutNew = std::move(initialCableLayout.first);
    auto cableLayoutGANew = std::move(initialCableLayout.second);

    auto cluster = initializeCluster();

    double cost = wf_.costTotal(*cableLayoutNew, *cableLayoutGANew);
    double cOld = cost +1;

    int counter = 0;
//     int numberOfSubstations =  wf_.substations_.size();
    int n = 1;
    while(counter < stop_ && n<3) {
        std::shuffle(wf_.turbines_.begin(), wf_.turbines_.end(), std::default_random_engine(std::random_device()()));
        if(cOld > cost) {
            n = 1;
            cOld = cost;
			double cTemp = cost;
            for(auto t: wf_.turbines_) {
                ogdf::SimpleCluster* clustT = graphalgos::getRootCluster(t->index(), cluster);

                ogdf::SimpleCluster* clustTemp = clustT;
                ogdf::node const parentNode = graphalgos::parent(t, *cableLayoutNew);

                auto Gtemp = std::make_unique<ogdf::Graph> (*cableLayoutNew);
                auto GATemp = std::make_unique<ogdf::GraphAttributes> (*Gtemp,
                              ogdf::GraphAttributes::nodeId |
                              ogdf::GraphAttributes::edgeIntWeight);

                //Fluss kopieren
                for(auto e: Gtemp->edges) {
                    ogdf::node sourceOrig = searchNode(e->source()->index(), *cableLayoutNew);
                    ogdf::node targetOrig = searchNode(e->target()->index(), *cableLayoutNew);
                    ogdf::edge eOrig = cableLayoutNew->searchEdge(sourceOrig, targetOrig);
                    GATemp->intWeight(e) = cableLayoutGANew->intWeight(eOrig);
                }

                //günstigste Kante zum Clustern suchen, falls sie existiert
                for(ogdf::adjEntry adj: t->adjEntries) {
                    auto n = adj->theEdge()->opposite(t);

                    auto flowNew = wf_.redirect(parentNode->index(), t->index() , n->index(), cost, *cableLayoutNew, *cableLayoutGANew);
                    if (!flowNew) {
                        continue;
                    }
//                     cTemp = wf_.costTotal(*flowNew->first, *flowNew->second);
					if (std::get<2>(*flowNew) < cTemp) {
						cTemp =std::get<2>(*flowNew);
                        clustTemp = graphalgos::getRootCluster(n->index(), cluster);
						GATemp = std::move(std::get<1>(*flowNew));
						Gtemp = std::move(std::get<0>(*flowNew));
                    }
                }
                if(clustTemp != clustT) {
                    graphalgos::mergeClusters(*clustTemp, *clustT, cluster);
                    cableLayoutNew = std::move(Gtemp);
                    cableLayoutGANew = std::move(GATemp);
					cost = cTemp;
					int pos = 0;
					auto it = std::find(wf_.turbines_.begin(), wf_.turbines_.end(), t);
					if (it == wf_.turbines_.end())
					{
						// name not in vector
					} else
					{
						pos = std::distance(wf_.turbines_.begin(), it);
					}
					std::cout<< "cost: " << cost << ", counter: " << counter << ", step: " << pos << "node: " << t << std::endl;
					wf_.cableToSVG("SingleMoveMover-" + std::to_string(counter) + "step-" + std::to_string(pos) + "-node-" + std::to_string(t->index()) + ".svg", *cableLayoutNew, *cableLayoutGANew);
					
                }

            }
        } else {
            n++;
            cOld = cost;
			double cTemp = cost;
            for(auto t: wf_.turbines_) {
                ogdf::SimpleCluster* clustT = graphalgos::getRootCluster(t->index(), cluster);

                ogdf::SimpleCluster* clustTemp = clustT;
                ogdf::node const parentNode = graphalgos::parent(t, *cableLayoutNew);

                auto Gtemp = std::make_unique<ogdf::Graph> (*cableLayoutNew);
                auto GATemp = std::make_unique<ogdf::GraphAttributes> (*Gtemp,
                              ogdf::GraphAttributes::nodeId |
                              ogdf::GraphAttributes::edgeIntWeight);

                //Fluss kopieren
                for(auto e: Gtemp->edges) {
                    ogdf::node sourceOrig = searchNode(e->source()->index(), *cableLayoutNew);
                    ogdf::node targetOrig = searchNode(e->target()->index(), *cableLayoutNew);
                    ogdf::edge eOrig = cableLayoutNew->searchEdge(sourceOrig, targetOrig);
                    GATemp->intWeight(e) = cableLayoutGANew->intWeight(eOrig);
                }
                for(ogdf::adjEntry adj: t->adjEntries) {
                    auto n = adj->theEdge()->opposite(t);

                    auto flowNew = wf_.redirectSingleNode(parentNode->index(), t->index() , n->index(), cost, *cableLayoutNew, *cableLayoutGANew);
                    if (!flowNew) {
                        continue;
                    }
//                     cTemp = wf_.costTotal(*flowNew->first, *flowNew->second);
                    if (std::get<2>(*flowNew) < cTemp) {
						cTemp = std::get<2>(*flowNew);
                        clustTemp = graphalgos::getRootCluster(n->index(), cluster);
						GATemp = std::move(std::get<1>(*flowNew));
						Gtemp = std::move(std::get<0>(*flowNew));

                    }
                }
                if(clustTemp != clustT) {
                    graphalgos::mergeClusters(*clustTemp, *clustT, cluster);
                    cableLayoutNew = std::move(Gtemp);
                    cableLayoutGANew = std::move(GATemp);
					cost = cTemp;					int pos = 0;
					auto it = std::find(wf_.turbines_.begin(), wf_.turbines_.end(), t);
					if (it == wf_.turbines_.end())
					{
						// name not in vector
					} else
					{
						pos = std::distance(wf_.turbines_.begin(), it);
					}
					std::cout<< "cost: " << cost << ", counter: " << counter << ", step: " << pos << "node: " << t << std::endl;
					wf_.cableToSVG("SingleMove-" + std::to_string(counter) + "step-" + std::to_string(pos) + "-node-" + std::to_string(t->index()) + ".svg", *cableLayoutNew, *cableLayoutGANew);
                }
            }
        }

        counter++;
    }
    return std::make_tuple(std::move(cableLayoutNew), std::move(cableLayoutGANew), cost, cluster);
}

ClusterResult WindfarmMover::adoptionMove() {
	auto initialCableLayout = wf_.initializeCableLayout();
    auto cableLayoutNew = std::move(initialCableLayout.first);
    auto cableLayoutGANew = std::move(initialCableLayout.second);

    auto cluster = initializeCluster();
    double cost = wf_.costTotal(*cableLayoutNew, *cableLayoutGANew);
    double cOld = cost +1;
	
	wf_.cableToSVG("initialLayout.svg", *cableLayoutNew, *cableLayoutGANew);
    int counter = 0;
//     int numberOfSubstations = wf_.substations_.size();
    int n = 1;
	
// 	for(auto e: cableLayoutNew->edges){
// 		std::cout << "Kante: " << e << ", Fluss: " << cableLayoutGANew->intWeight(e) << std::endl;
// 	}
	
    while(counter < stop_ && n<3) {
        std::shuffle(wf_.turbines_.begin(), wf_.turbines_.end(), std::default_random_engine(std::random_device()()));
        if(cOld > cost) {
            n = 1;
            cOld = cost;

            for(auto t: wf_.turbines_) {
				double cTemp = cost;
                ogdf::SimpleCluster* clustT = graphalgos::getRootCluster(t->index(), cluster);

                ogdf::SimpleCluster* clustTemp = clustT;
                ogdf::node const parentNode = graphalgos::parent(t, *cableLayoutNew);

                auto Gtemp = std::make_unique<ogdf::Graph> (*cableLayoutNew);
                auto GATemp = std::make_unique<ogdf::GraphAttributes> (*Gtemp,
                              ogdf::GraphAttributes::nodeId |
                              ogdf::GraphAttributes::edgeIntWeight);

                //Fluss kopieren
                for(auto e: Gtemp->edges) {
                    ogdf::node sourceOrig = searchNode(e->source()->index(), *cableLayoutNew);
                    ogdf::node targetOrig = searchNode(e->target()->index(), *cableLayoutNew);
                    ogdf::edge eOrig = cableLayoutNew->searchEdge(sourceOrig, targetOrig);
                    GATemp->intWeight(e) = cableLayoutGANew->intWeight(eOrig);
                }
                

                //günstigste Kante zum Clustern suchen, falls sie existiert
                for(ogdf::adjEntry adj: t->adjEntries) {

                    auto n = adj->theEdge()->opposite(t);
				
                    auto flowNew = wf_.redirect(parentNode->index(), t->index() , n->index(), cost, *cableLayoutNew, *cableLayoutGANew);
                    if (!flowNew) {
                        continue;
                    }
//                     cTemp = wf_.costTotal(*flowNew->first, *flowNew->second);
						std::cout << "counter: " << counter << ", Knoten T: " << t << ", neuer Elter: " << n << ", cost: " << cost << ", ctemp: " << cTemp << ", neuer Fluss; " << std::get<2>(*flowNew) << std::endl;
                    if (std::get<2>(*flowNew) < cTemp) {
						cTemp = std::get<2>(*flowNew);
                        clustTemp = graphalgos::getRootCluster(n->index(), cluster);
						GATemp = std::move(std::get<1>(*flowNew));
						Gtemp = std::move(std::get<0>(*flowNew));
                    }
                    
                }
                if(cTemp < cost) {
                    graphalgos::mergeClusters(*clustTemp, *clustT, cluster);
                    cableLayoutNew = std::move(Gtemp);
                    cableLayoutGANew = std::move(GATemp);
					cost = cTemp;					int pos = 0;
					auto it = std::find(wf_.turbines_.begin(), wf_.turbines_.end(), t);
					if (it == wf_.turbines_.end())
					{
						// name not in vector
					} else
					{
						pos = std::distance(wf_.turbines_.begin(), it);
					}
// 					std::cout<< "cost: " << cost << ", counter: " << counter << ", step: " << pos << "node: " << t << std::endl;
					wf_.cableToSVG("AdoptionMover-" + std::to_string(counter) + "step-" + std::to_string(pos) + "-node-" + std::to_string(t->index()) + ".svg", *cableLayoutNew, *cableLayoutGANew);
					
                }

            }
        } else {
            n++;
            cOld = cost;
			
            for(auto t: wf_.turbines_) {
				double cTemp = cost;
                ogdf::SimpleCluster* clustT = graphalgos::getRootCluster(t->index(), cluster);

                ogdf::SimpleCluster* clustTemp = clustT;
                ogdf::node const parentNode = graphalgos::parent(t, *cableLayoutNew);

                auto Gtemp = std::make_unique<ogdf::Graph> (*cableLayoutNew);
                auto GATemp = std::make_unique<ogdf::GraphAttributes> (*Gtemp,
                              ogdf::GraphAttributes::nodeId |
                              ogdf::GraphAttributes::edgeIntWeight);

                //Fluss kopieren
                for(auto e: Gtemp->edges) {
                    ogdf::node sourceOrig = searchNode(e->source()->index(), *cableLayoutNew);
                    ogdf::node targetOrig = searchNode(e->target()->index(), *cableLayoutNew);
                    ogdf::edge eOrig = cableLayoutNew->searchEdge(sourceOrig, targetOrig);
                    GATemp->intWeight(e) = cableLayoutGANew->intWeight(eOrig);
                }
                

	
                
                for(ogdf::adjEntry adj: t->adjEntries) {
                    auto n = adj->theEdge()->opposite(t);

                    std::set<int> childrenN;
                    for(auto c: graphalgos::children(n, *Gtemp)) {
						if(wf_.getDistanceTo(c, n) >= wf_.getDistanceTo(c, t)){
	                        childrenN.emplace(c->index());
						}
                    }


//                     std::cout << "childrenN.size() = " << childrenN.size() << std::endl;
                    if(childrenN.size()> wf_.nC_ -1) {
                        continue;
                    }

                    for(auto s: subsets<int>(childrenN)) {

                        auto flowNew = wf_.redirectAdoption(parentNode->index(), t->index() , n->index(), s, cost,*cableLayoutNew, *cableLayoutGANew);
                        if (!flowNew) {
                            continue;
                        }
                        assert(equalDoubleDelta(std::get<2>(*flowNew), wf_.costTotal(*std::get<0>(*flowNew), *std::get<1>(*flowNew))));

                        if (std::get<2>(*flowNew) < cTemp) {
							cTemp = std::get<2>(*flowNew);
							std::cout << "Adoptionspart: Knoten in";
							for(auto x: s){
								std::cout << x << ", ";
							}
							std::cout << " adoptiert von Knoten " << t << ", cTemp: " << cTemp << ", cost: " << cost << std::endl;
                            clustTemp = graphalgos::getRootCluster(n->index(), cluster);
							std::cout << "clustTemp: " << clustTemp->getIndex() << ", clustT: " << clustT->getIndex() << std::endl;
                            GATemp = std::move(std::get<1>(*flowNew));
                            Gtemp = std::move(std::get<0>(*flowNew));
                        }
                    }
                }
                if(cTemp < cost) {
                    graphalgos::mergeClusters(*clustTemp, *clustT, cluster);
                    cableLayoutNew = std::move(Gtemp);
                    cableLayoutGANew = std::move(GATemp);
					cost = cTemp;
					int pos = 0;
					auto it = std::find(wf_.turbines_.begin(), wf_.turbines_.end(), t);
					if (it == wf_.turbines_.end())
					{
						// name not in vector
					} else
					{
						pos = std::distance(wf_.turbines_.begin(), it);
					}
					std::cout<< "cost: " << cost << ", counter: " << counter << ", step: " << pos << "node: " << t << std::endl;
					wf_.cableToSVG("adoptionZ-" + std::to_string(counter) + "step-" + std::to_string(pos) + "-node-" + std::to_string(t->index()) + ".svg", *cableLayoutNew, *cableLayoutGANew);
                }
            }
        }
// 		std::cout << "counter: " << counter << ", cost: " << cost << std::endl;
        counter++;
    }
    cost = wf_.costTotal(*cableLayoutNew, *cableLayoutGANew);
	wf_.cableLayout_ = std::move(cableLayoutNew);
	wf_.cableLayoutGA_ = std::move(cableLayoutGANew);
    return std::make_tuple(std::move(cableLayoutNew), std::move(cableLayoutGANew), cost, cluster);
}

void WindfarmMover::findBest(ClusterFunc func) {
    int counter = 0;
    while(counter < 30) {
        auto result = func();
        double cost = std::get<2>(result);
        auto cluster = std::get<3>(result);
         std::cout << "findBest: counter= " << counter << ", cost = " << cost << ", cost_total_ = " << cost_total_ << std::endl;
        if(cost < cost_total_) {
            wf_.cableLayout_ = std::move(std::get<0>(result) );
            wf_.cableLayoutGA_ = std::move(std::get<1>(result) );
            cost_total_ = cost;
            for(auto c: cluster_) {
                delete c;
            }
            cluster_ = cluster;
            counter = 0;
// 			double costTest = wf_.costTotal(*wf_.cableLayout_, *wf_.cableLayoutGA_);


        } else {
            counter ++;
            for(auto c: cluster) {
                delete c;
            }
        }

    }
}


