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

#include "multilevelqualitythreshold.h"

MultiLevelQualityThreshold::MultiLevelQualityThreshold(Windfarm& wf) : wf_(wf) {}

std::set<std::set<ogdf::node> > MultiLevelQualityThreshold::qualityThresholdClusteringDistance(std::set<ogdf::node>& sites, double threshold, u_int minSize) {
    std::set<std::set<ogdf::node>> candidateCluster;
    std::set<std::set<ogdf::node>> foundCluster;
    std::set<ogdf::node> workingList = sites;

    std::set<ogdf::node> maxCandidate = workingList;

    while(maxCandidate.size() > minSize) {
        candidateCluster.clear();

        //build candidateClusters for all nodes
        for(auto v: workingList) {
            std::set<ogdf::node> vClust {v};
            for(auto w: workingList) {
                if(w == v) {
                    continue;
                }
                if(wf_.getDistanceTo(v, w) < threshold) {
                    vClust.emplace(w);
                }
            }
            if(vClust.size() > minSize) {
                candidateCluster.emplace(vClust);
            }
        }
        //search the biggest candidate Cluster
        maxCandidate.clear() ;
        for(auto s: candidateCluster) {
            if(s.size() > maxCandidate.size()) {
                maxCandidate = s;
            }
        }
        if(maxCandidate.size() > minSize) {
            foundCluster.emplace(maxCandidate);
        }

        //delete all the nodes in maxCandidate from the workingList
        for(auto v: maxCandidate) {
            workingList.erase(v);
        }
    }

    //all not clustered turbines are clusters with size 1
    for(auto w: workingList) {
        std::set<ogdf::node> wClust = {w};
        foundCluster.emplace(wClust);
    }
    return foundCluster;
}

std::set<std::set<ogdf::node> > MultiLevelQualityThreshold::qualityThresholdClusteringCost(std::map<ogdf::node, int>& sites, double threshold, u_int minSize) {
    std::set<std::set<ogdf::node>> candidateCluster;
    std::set<std::set<ogdf::node>> foundCluster;
    std::map<ogdf::node, int> workingList = sites;

    std::set<ogdf::node> maxCandidate;
    for(auto t: workingList) {
        maxCandidate.emplace(t.first);
    }

    while(maxCandidate.size() > minSize) {
        candidateCluster.clear();

        //build candidateClusters for all nodes
        for(auto v: workingList) {
            std::set<ogdf::node> vClust {v.first};
            for(auto w: workingList) {
                if(w == v) {
                    continue;
                }
                if(wf_.getDistanceTo(v.first, w.first)* wf_.cableCostPerUnit(v.second) < threshold) {
                    vClust.emplace(w.first);
                }
            }
            if(vClust.size() > minSize) {
                candidateCluster.emplace(vClust);
            }
        }
        //search the biggest candidate Cluster
        maxCandidate.clear() ;
        for(auto s: candidateCluster) {
            if(s.size() > maxCandidate.size()) {
                maxCandidate = s;
            }
        }
        if(maxCandidate.size() > minSize) {
            foundCluster.emplace(maxCandidate);
        }

        //delete all the nodes in maxCandidate from the workingList
        for(auto v: maxCandidate) {
            workingList.erase(v);
        }
    }
    //all not clustered turbines are clusters with size 1
    for(auto w: workingList) {
        std::set<ogdf::node> wClust = {w.first};
        foundCluster.emplace(wClust);
    }

    return foundCluster;
}


std::pair<std::unique_ptr<ogdf::Graph>, double> MultiLevelQualityThreshold::multiLevelClusteringDistance(double initialMultiplyer) {
    auto G = std::make_unique<ogdf::Graph> (wf_.getGraph());
    ogdf::List<ogdf::edge> edges;
    G->allEdges(edges);
    for(auto e: edges) {
        G->delEdge(e);
    }

    double threshold = initialMultiplyer * wf_.getMinDistance();

    std::set<ogdf::node> workingList;
    for(auto t: wf_.turbines_) {
        workingList.emplace(t);
    }

    std::map<ogdf::node, std::set<ogdf::node> > representatives;
    for(auto v: workingList) {
        std::set<ogdf::node> vSet = {v};
        representatives.emplace(v, vSet);
    }

    std::set<ogdf::node> maxCluster;

    while(maxCluster.size() <= wf_.nC_ && threshold < 100000) {

        std::set<std::set<ogdf::node>> cluster = qualityThresholdClusteringDistance(workingList, threshold, MINSIZE);

        for(auto c: cluster) {
            //find representatives
            ogdf::node rep;
            double distance = __INT_MAX__;
            for(auto v: c) {
                ogdf::node substation = wf_.getClosestSubstation(v);
                double d = wf_.getDistanceTo(v, substation);
                if(d <= distance) {
                    distance = d;
                    rep = v;
                }
            }

            //connect all nodes in c with rep in a star form
            for(auto v: c) {
                if(v == rep) {
                    continue;
                }
                G->newEdge(searchNode(rep->index(), *G), searchNode(v->index(), *G));
            }

            //erase all nodes in c excepts rep from the workingList
            for(auto v: c) {
                if(v == rep) {
                    continue;
                }
                workingList.erase(v);
            }

            std::set<ogdf::node> clust;
            for(auto v: c) {
                for(auto x: representatives.at(v)) {
                    clust.insert(x);
                }
                representatives.erase(v);
            }
            representatives.emplace(rep, clust);
        }
        maxCluster.clear();
        for(auto c: representatives) {
            if(c.second.size() > maxCluster.size()) {
                maxCluster = c.second;
            }
        }
        threshold = 2* threshold;
    }
    for(auto t: wf_.turbines_) {
        ogdf::node v = searchNode(t->index(), *G);
        if(v->indeg() == 0) {
            ogdf::node substation = wf_.getClosestSubstation(v);
            G->newEdge(searchNode(substation->index(), *G), v);
        }
    }
    *wf_.cableLayout_ = *G;
    wf_.computeFlow();
    return std::make_pair(std::move(G), wf_.costTotal(*wf_.cableLayout_, *wf_.cableLayoutGA_));
}

std::pair<std::unique_ptr<ogdf::Graph>, double> MultiLevelQualityThreshold::multiLevelClusteringCost(double initialMultiplyer) {
    auto G = std::make_unique<ogdf::Graph> (wf_.getGraph());
    ogdf::List<ogdf::edge> edges;
    G->allEdges(edges);
    for(auto e: edges) {
        G->delEdge(e);
    }
    double threshold = initialMultiplyer * wf_.getMinDistance() *wf_.cableCostPerUnit(1);

    //maps all turbines to cluster to the size of its cluster
    std::map<ogdf::node, int> workingList;
    for(auto t: wf_.turbines_) {
        workingList.emplace(t, 1);
    }

    ogdf::SList<ogdf::SimpleCluster*> cluster;
    for(auto v: wf_.getGraph().nodes) {
        ogdf::SimpleCluster* clust = new ogdf::SimpleCluster;
        clust->setIndex(v->index());
        clust->pushBackVertex(v);
        cluster.pushBack(clust);
    }
    int maxClusterSize = 0;

    while(maxClusterSize < wf_.nC_ && threshold < 100000) {

        std::set<std::set<ogdf::node>> foundCluster = qualityThresholdClusteringCost(workingList, threshold, MINSIZE);

        for(auto c: foundCluster) {

            //find representative of the cluster which is nearest to a substation
            ogdf::node rep;
            double distance = __INT_MAX__;

            for(auto v: c) {
                ogdf::node substation = wf_.getClosestSubstation(v);
                double d = wf_.getEuklideanDistance(v, substation);
                if(d <= distance) {
                    distance = d;
                    rep = v;
                }
            }
            ogdf::SList<ogdf::SimpleCluster*> clusterList;
            for(auto v: c) {
                ogdf::SimpleCluster* vC = graphalgos::searchCluster(v->index(), cluster);
                clusterList.pushBack(vC);
            }
            if(clusterList.size() > 1) {
                graphalgos::mergeClusters(clusterList, cluster);

                //construct a subgraph induced by this cluster
                ogdf::Graph subgraph = graphalgos::subGraph(c, wf_.getGraph());
                ogdf::EdgeArray<double> weight(subgraph);
                for(auto e: subgraph.edges) {
                    ogdf::node t = searchNode(e->target()->index(), wf_.getGraph());

                    weight[e] = workingList.at(t)*wf_.getDistanceTo(t, e->source());
                }
                if(!ogdf::isConnected(subgraph)) {
                    std::cout << "subgraph not connected" << std::endl;
                }
                ogdf::EdgeArray<bool> isInTree(subgraph);
                ogdf::computeMinST(subgraph,weight, isInTree);

                std::vector<ogdf::edge> toDelete;
                for( auto e: subgraph.edges) {
                    if(!isInTree[e]) {
                        toDelete.emplace_back(e);
                    }
                }
                for(auto e: toDelete) {
                    subgraph.delEdge(e);
                }
                if(!graphalgos::toArborescene(subgraph, searchNode(rep->index(), subgraph))) {
                    std::cout << "in arborescence orientieren nicht erfolgreich" <<std::endl;
                }

                for(auto e: subgraph.edges) {
                    ogdf::node t = searchNode(e->target()->index(), *G);
                    ogdf::node s = searchNode(e->source()->index(), *G);
                    G->newEdge(s, t);
                }
            }
            //erase all nodes in c excepts rep from the workingList
            for(auto v: c) {
                if(v == rep) {
                    continue;
                }
                workingList.erase(v);
            }
            //update the entry at rep to have the size of the cluster
            int size = graphalgos::clusterSize(graphalgos::searchCluster(rep->index(), cluster));
            workingList.at(rep) = size;
        }

        maxClusterSize = 0;
        for(auto c: cluster) {
            int size = graphalgos::clusterSize(c);
            if(size > maxClusterSize) {
                maxClusterSize = size;
            }
        }
        threshold = 2*threshold;
    }

    for(auto t: wf_.turbines_) {
        ogdf::node v = searchNode(t->index(), *G);
        if(v->indeg() == 0) {
            ogdf::node nearestNeighbour = findNearestValidNode(v, cluster, *G);
            if(!nearestNeighbour) {
                std::cout << "keinen gültigen nachbarn gefunden!" <<std::endl;
            } else {
                G->newEdge(searchNode(nearestNeighbour->index(), *G), v);
            }
        }
    }

    *wf_.cableLayout_ = *G;
    wf_.computeFlow();
    for (auto c: cluster)
        delete c;
    return std::make_pair(std::move(G), wf_.costTotal(*wf_.cableLayout_, *wf_.cableLayoutGA_));
}


void MultiLevelQualityThreshold::findBestInitialThresholdDistance() {
    double cost = __INT_MAX__;
    cost = cost * 10000;
    auto G = std::make_unique<ogdf::Graph> ();
    for(double m = 1; m < 30; m += 0.5 ) {
        auto result = multiLevelClusteringDistance(m);
        double costTemp = result.second;
        if(costTemp <= cost) {
            cost = costTemp;
            G = std::move(result.first);
        }
    }
    wf_.cableLayout_ = std::move(G);
    wf_.computeFlow();
}

void MultiLevelQualityThreshold::findBestInitialThresholdCost() {
    double cost = __INT_MAX__;
    cost = cost * 10000;
    auto G = std::make_unique<ogdf::Graph> ();
    for(double m = 1; m < 30; m += 0.5 ) {
        auto result = multiLevelClusteringCost(m);
        double costTemp = result.second;
        if(costTemp <= cost) {
            cost = costTemp;
            G = std::move(result.first);
        }
        std::cout << "m: " << m << ", cost: " << cost << ", costTemp: " << costTemp << std::endl;
    }
    wf_.cableLayout_ = std::move(G);
    wf_.computeFlow();
}

ogdf::node MultiLevelQualityThreshold::findNearestValidNode(ogdf::node v, ogdf::SList<ogdf::SimpleCluster *> cluster, ogdf::Graph& G) {
    ogdf::node result = wf_.getClosestSubstation(v);
    double distance = __INT_MAX__;
    ogdf::node vOrig = searchNode(v->index(), wf_.getGraph());
    auto vRoot = graphalgos::getRootCluster(v->index(), cluster);
    int vSize = vRoot->nodes().size();
	ogdf::NodeArray< int > component(G);
	ogdf::connectedComponents(G, component);

    for(auto adj: vOrig->adjEntries) {
        ogdf::node n = adj->theEdge()->opposite(vOrig);
        double d = wf_.getDistanceTo(v, n);
		if(d < distance && component[v] != component[searchNode(n->index(), G)] ) {
            ogdf::SimpleCluster* clust = graphalgos::getRootCluster(n->index(), cluster);
            if (clust->nodes().size() + vSize <= wf_.nC_) {
                result = n;
                distance = d;
            }
        }
    }
    return result;
}


