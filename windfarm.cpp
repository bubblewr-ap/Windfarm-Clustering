#include <ogdf/basic/simple_graph_alg.h>
#include <ogdf/basic/simple_graph_alg.h>
#include <ogdf/basic/simple_graph_alg.h>
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

#include "windfarm.h"

Windfarm::Windfarm(std::string filename, int nS, int nC) : nS_(nS), nC_(nC)  {
    //Graph einlesen von GML-Datei
    topoGA_.init(topoG_,
                 ogdf::GraphAttributes::nodeGraphics |
                 ogdf::GraphAttributes::edgeGraphics |
                 ogdf::GraphAttributes::nodeLabel |
                 ogdf::GraphAttributes::edgeStyle |
                 ogdf::GraphAttributes::nodeStyle |
                 ogdf::GraphAttributes::nodeTemplate |
                 ogdf::GraphAttributes::nodeId |
                 ogdf::GraphAttributes::edgeDoubleWeight);

    if (!ogdf::GraphIO::read(topoGA_, topoG_, filename, ogdf::GraphIO::readGML)) {
        throw std::invalid_argument("Could not load " +  filename);
    }
	
    //Kantenlänge als Kantengewicht setzen
    for (auto e: topoG_.edges) {
        ogdf::node source = (e->source());
        ogdf::node target = (e->target());

        double sourceX = topoGA_.x(source);
        double sourceY = topoGA_.y(source);
        double targetX = topoGA_.x(target);
        double targetY = topoGA_.y(target);
        double weight = std::hypot((sourceX-targetX), (sourceY-targetY));

        topoGA_.doubleWeight(e) = weight;
    }

    ogdf::GraphIO::write(topoGA_, "input.svg", ogdf::GraphIO::drawSVG);
    //turbines_ und substations_ initialisieren
    for (auto v: topoG_.nodes) {
        if(topoGA_.fillColor(v).toString()== "#6A5ACD") {
            turbines_.emplace_back(v);
        } else {
            substations_.emplace_back(v);
        }
    }

    std::cout << "nS = " << nS_ << ", |V_T| = " << turbines_.size() << ", |V_S| = " << substations_.size() << std::endl;
	
    //initiale Zuordnung Turbinen an Substations
    auto result = initializeCableLayout();
    cableLayout_ = std::move(result.first);
    cableLayoutGA_ = std::move(result.second);
}

std::pair<std::unique_ptr<ogdf::Graph>, std::unique_ptr<ogdf::GraphAttributes> > Windfarm::initializeCableLayout() {
    auto cableLayout = std::make_unique<ogdf::Graph>(topoG_);
    auto cableLayoutGA = std::make_unique<ogdf::GraphAttributes> (*cableLayout,
                         ogdf::GraphAttributes::nodeId |
                         ogdf::GraphAttributes::edgeIntWeight);

    int turbinesPerSubstation = turbines_.size() / (substations_.size());


    if(turbinesPerSubstation > nS_  ) {
        throw std::invalid_argument("too many turbines");
    } else if  (turbinesPerSubstation == nS_ && (turbines_.size() % (substations_.size()) != 0)) {
        throw std::invalid_argument("too many turbines");
    }

    std::vector<ogdf::edge> toDelete;

     std::shuffle(turbines_.begin(), turbines_.end(), std::default_random_engine(std::random_device()()));
    for(u_int i = 0; i < turbines_.size(); ++i) {
        ogdf::node substation = graphalgos::searchNode(substations_[i % substations_.size()]->index(), *cableLayout);
        ogdf::node v = graphalgos::searchNode(turbines_[i]->index(), *cableLayout);
        if(v == nullptr) {
            //std::cout << "Node existiert nicht" << std::endl;
        }
        bool edgeExist = false;
        for(auto adj: v->adjEntries) {
            ogdf::edge e = adj->theEdge();
            ogdf::node n = e->opposite(v);
            if(n == substation) {
                edgeExist = true;
                cableLayoutGA->intWeight(e) = 1;

                //Kanten sollen immer von der Wurzel weggerichtet sein
                if(e->target() != v) {
                    cableLayout->reverseEdge(e);
                }
            } else {
                if(std::find(std::begin(toDelete), std::end(toDelete), e) == std::end(toDelete)) {
                    toDelete.emplace_back(e);
                }
            }
        }
        if(!edgeExist) {
            ogdf::edge e = cableLayout->newEdge(substation, v);
            cableLayoutGA->intWeight(e) = 1;
        }
    }
    for(auto e: toDelete) {
        cableLayout->delEdge(e);
    }

    return std::make_pair(std::move(cableLayout), std::move(cableLayoutGA));
}

std::pair<std::unique_ptr<ogdf::Graph>, std::unique_ptr<ogdf::GraphAttributes> > Windfarm::initializeCableLayoutDistance() {
    auto cableLayout = std::make_unique<ogdf::Graph>(topoG_);
    auto cableLayoutGA = std::make_unique<ogdf::GraphAttributes> (*cableLayout,
                         ogdf::GraphAttributes::nodeId |
                         ogdf::GraphAttributes::edgeIntWeight);
    //alle Kanten aus cableLayout löschen
    ogdf::List<ogdf::edge> edges;
    cableLayout->allEdges(edges);
    for(auto e: edges) {
        cableLayout->delEdge(e);
    }
    int turbinesPerSubstation = turbines_.size() / (substations_.size());

    if(turbinesPerSubstation > nS_  ) {
        throw std::invalid_argument("too many turbines");
    } else if  (turbinesPerSubstation == nS_ && (turbines_.size() % (substations_.size()) != 0)) {
        throw std::invalid_argument("too many turbines");
    }

    struct DistanceTo {
        DistanceTo(Windfarm const& wf, ogdf::node trafostation) : wf(wf), trafostation(trafostation) {}

        Windfarm const& wf;
        ogdf::node trafostation;

        bool operator()(ogdf::node const& a, ogdf::node const& b) const {
            return wf.getEuklideanDistance(trafostation, a) < wf.getEuklideanDistance(trafostation, b);
        }
    };

    std::map<ogdf::node, std::vector<ogdf::node>> nearLists;

    for (unsigned i = 0; i < substations_.size(); ++i) {
        DistanceTo comp(*this, substations_[i]);
        nearLists.emplace(substations_[i], std::vector<ogdf::node>(turbines_.begin(), turbines_.end()));
        std::sort(nearLists.at(substations_[i]).begin(), nearLists.at(substations_[i]).end(), comp);
    }

    std::set<ogdf::node> workedNodes;

    while(workedNodes.size() < turbines_.size()) {
        std::shuffle(substations_.begin(), substations_.end(), std::default_random_engine(std::random_device()()));
        for(auto s: substations_) {
            if(nearLists.at(s).size() > 0) {
                ogdf::node nearestNode = nearLists.at(s).front();
                while(workedNodes.find(nearestNode)  != workedNodes.end()) {
                    nearLists.at(s).erase(nearLists.at(s).begin());
					if(nearLists.at(s).empty()) {
						break;
					}
                    nearestNode = nearLists.at(s).front();
                }
                if(nearLists.at(s).empty()) {
                    continue;
                }
                ogdf::edge e = cableLayout->newEdge(searchNode(s->index(), *cableLayout), searchNode(nearestNode->index(), *cableLayout));
                cableLayoutGA->intWeight(e) = 1;
                workedNodes.emplace(nearestNode);
            }
        }
    }
    return std::make_pair(std::move(cableLayout), std::move(cableLayoutGA));
}

RedirectResult Windfarm::redirect(int oldParent, int u, int newParent, double costOld, ogdf::Graph const& G, ogdf::GraphAttributes const& GA) {
    auto Gnew = std::make_unique<ogdf::Graph> (G);
    auto GAnew = std::make_unique<ogdf::GraphAttributes> (*Gnew,
                 ogdf::GraphAttributes::nodeId |
                 ogdf::GraphAttributes::edgeIntWeight);

    auto uNode = searchNode(u, *Gnew);
    auto oldParentNode = searchNode(oldParent, *Gnew);
    auto newParentNode = searchNode(newParent, *Gnew);

    double cost = costOld;

    //Fluss kopieren
    for(auto e: Gnew->edges) {
        ogdf::node sourceOrig = searchNode(e->source()->index(), G);
        ogdf::node targetOrig = searchNode(e->target()->index(), G);
        ogdf::edge eOrig = G.searchEdge(sourceOrig, targetOrig);
        GAnew->intWeight(e) = GA.intWeight(eOrig);
    }

    ogdf::edge e_uOldParent = Gnew->searchEdge(uNode, oldParentNode);

    int flowdiff = GAnew->intWeight(e_uOldParent);
    cost -= getDistanceTo(oldParentNode, uNode) * cableCostPerUnit(flowdiff);
    Gnew->delEdge(e_uOldParent);

    ogdf::edge e_uNewParent = Gnew->newEdge(newParentNode, uNode);
    GAnew->intWeight(e_uNewParent) = flowdiff;
    cost += getDistanceTo(uNode,  newParentNode)  * cableCostPerUnit(flowdiff);
    if(!isForest(*Gnew)) {
        return boost::none;
    }

    //Flowdiff abziehen von dem alten Pfad
    ogdf::node it = oldParentNode;
    while(isTurbine(it)) {
        auto p = graphalgos::parent(it, *Gnew);
        assert(p);
//         p = searchNode(p->index(), *Gnew);
        ogdf::edge e = Gnew->searchEdge(it, p);
        if(!e) {
            throw std::domain_error( "Kante " + std::to_string(it->index()) + "->" + std::to_string(p->index()) + " existiert nicht! Sollte nicht passieren");

        }
        double distance = getDistanceTo(it, p);
        double costOlde = distance * cableCostPerUnit(GAnew->intWeight(e));
        GAnew->intWeight(e) -= flowdiff;
// 		std::cout << "in redirect Schritt Flow abziehen, Kante: " << e << ", Fluss: " <<  GAnew->intWeight(e) << std::endl;
		if(GAnew->intWeight(e) < 0){
			std::cout << " in redirect: Kante: " << e << ", Fluss: " << GAnew->intWeight(e) << std::endl;
			throw std::domain_error("negatives Kantengewicht");
		}
        double costNewe = distance * cableCostPerUnit(GAnew->intWeight(e));
        cost = cost - costOlde + costNewe;
        it = p;
    }

    //flowdiff auf den neuen Pfad addieren
    it = newParentNode;
    while(isTurbine(it)) {
        ogdf::node p = graphalgos::parent(it, *Gnew);
        ogdf::edge e = Gnew->searchEdge(it, p);
        if(!e) {
            throw std::domain_error("Kante " + std::to_string(it->index()) + "->" + std::to_string(p->index()) + " existiert nicht! Sollte nicht passieren");
        }
        double distance = getDistanceTo(it, p);
        double costOlde = distance * cableCostPerUnit(GAnew->intWeight(e));
        GAnew->intWeight(e) += flowdiff;
        int flowNew = GAnew->intWeight(e);
        double costNewe = distance * cableCostPerUnit(flowNew);
        if(isTurbine(p) && flowNew >= nS_) {
            return boost::none;
        } else if (!isTurbine(p) && flowNew >= nC_) {
            return boost::none;
        }
        cost = cost - costOlde + costNewe;
        it = p;
    }
    return std::make_tuple(std::move(Gnew), std::move(GAnew), cost);
    //printCableLayout();
}

RedirectResult Windfarm::redirectSingleNode(int oldParent, int u, int newParent, double costOld, ogdf::Graph const& G, ogdf::GraphAttributes const& GA) {
    auto Gnew = std::make_unique<ogdf::Graph> (G);
    auto GAnew = std::make_unique<ogdf::GraphAttributes> (*Gnew,
                 ogdf::GraphAttributes::nodeId |
                 ogdf::GraphAttributes::edgeIntWeight);

    if (oldParent == newParent)
        return boost::none;

    auto uNode = searchNode(u, *Gnew);
    auto oldParentNode = searchNode(oldParent, *Gnew);
    auto newParentNode = searchNode(newParent, *Gnew);

    double cost = costOld;
    if(!Gnew->searchEdge(uNode, oldParentNode)) {
        throw std::domain_error("Kante " + std::to_string(u) + "->" + std::to_string(oldParent) + " existiert in cableLayout_ nicht");
        return boost::none;
    }

    //Fluss kopieren
    for(auto e: Gnew->edges) {
        ogdf::node sourceOrig = searchNode(e->source()->index(), G);
        ogdf::node targetOrig = searchNode(e->target()->index(), G);
        ogdf::edge eOrig = G.searchEdge(sourceOrig, targetOrig);
        GAnew->intWeight(e) = GA.intWeight(eOrig);
    }

    auto childrenU = graphalgos::children(uNode, *Gnew);

    ogdf::edge e_uOldParent = Gnew->searchEdge(oldParentNode, uNode);
    cost -= getDistanceTo(oldParentNode, uNode) * cableCostPerUnit(GAnew->intWeight(e_uOldParent));
    Gnew->delEdge(e_uOldParent);
    ogdf::edge e_uNewParent = Gnew->newEdge(newParentNode, uNode);
    GAnew->intWeight(e_uNewParent) = 1;
    cost += getDistanceTo(uNode, newParentNode) * cableCostPerUnit(1);

    //Flowdiff abziehen von dem alten Pfad
    ogdf::node it = oldParentNode;
    while(isTurbine(it)) {
        auto p = graphalgos::parent(it, G);
        assert(p);
        p = searchNode(p->index(), *Gnew);
        ogdf::edge e = Gnew->searchEdge(p, searchNode(it->index(), *Gnew));
        if(!e) {
            throw std::domain_error("Kante existiert nicht! Sollte nicht passieren");
        }
        double distance = getDistanceTo(it, p);
        double costOlde = distance * cableCostPerUnit(GAnew->intWeight(e));
        GAnew->intWeight(e) -= 1;
        double costNewe = distance * cableCostPerUnit(GAnew->intWeight(e));
        cost = cost - costOlde + costNewe;
        it = p;
    }

    //alle Kinder werden von oldParent "adoptiert", also direkt verbunden
    for(auto c: childrenU) {
        if(c->index() == newParent) {
            continue;
        }

        auto eOld = Gnew->searchEdge(uNode, c);
        assert(eOld);

        int flow = GAnew->intWeight(eOld);
        cost -= getDistanceTo(uNode, c) * cableCostPerUnit(flow);
        Gnew->delEdge(eOld);
        auto eNew = Gnew->newEdge(oldParentNode, c);
        GAnew->intWeight(eNew) = flow;
        cost += getDistanceTo(oldParentNode, c) * cableCostPerUnit(flow);
    }

    it = newParentNode;
    while(isTurbine(it)) {
        ogdf::node p = graphalgos::parent(it, *Gnew);
        if(!p) {
            //std::cout << "hat kein Elter" << std::endl;
        }
        ogdf::edge e = Gnew->searchEdge(p, it);
        if(!e) {
            throw std::domain_error("Kante existiert nicht! Sollte nicht passieren");
        }
        double distance = getDistanceTo(it, p);
        double costOlde = distance * cableCostPerUnit(GAnew->intWeight(e));
        GAnew->intWeight(e) += 1;
        int flowNew = GAnew->intWeight(e);
        if(isTurbine(p) && flowNew >= nS_) {
            return boost::none;
        } else if (!isTurbine(p) && flowNew >= nC_) {
            return boost::none;
        }
        cost = cost - costOlde + distance * cableCostPerUnit(flowNew);
        it = p;
    }

    return std::make_tuple(std::move(Gnew), std::move(GAnew), cost);
}



RedirectResult Windfarm::redirectAdoption(int oldParent, int u, int newParent, std::set< int > childrenToAdopt, double costOld, const ogdf::Graph& G, const ogdf::GraphAttributes& GA)
{

    auto Gnew = std::make_unique<ogdf::Graph> (G);
    auto GAnew = std::make_unique<ogdf::GraphAttributes> (*Gnew,
                 ogdf::GraphAttributes::nodeId |
                 ogdf::GraphAttributes::edgeIntWeight);

    double cost = costOld;

    if (oldParent == newParent) {
        return boost::none;
    }

    auto uNode = searchNode(u, *Gnew);
    auto oldParentNode = searchNode(oldParent, *Gnew);
    auto newParentNode = searchNode(newParent, *Gnew);

    if(graphalgos::parent(newParentNode, *Gnew) == uNode) {
        return boost::none;
    }

    if(!Gnew->searchEdge(uNode, oldParentNode)) {
        throw std::domain_error("Kante " + std::to_string(u) + "->" + std::to_string(oldParent) + " existiert in cableLayout_ nicht");
        return boost::none;
    }

    //Fluss kopieren
    for(auto e: Gnew->edges) {
        ogdf::node sourceOrig = searchNode(e->source()->index(), G);
        ogdf::node targetOrig = searchNode(e->target()->index(), G);
        ogdf::edge eOrig = G.searchEdge(sourceOrig, targetOrig);
        GAnew->intWeight(e) = GA.intWeight(eOrig);
    }
    assert(graphalgos::parent(uNode, *Gnew) == oldParentNode);
    auto childrenU = graphalgos::children(uNode, *Gnew);

    ogdf::edge e_uOldParent = Gnew->searchEdge(oldParentNode, uNode);
    cost -= getDistanceTo(oldParentNode, uNode) * cableCostPerUnit(GAnew->intWeight(e_uOldParent));

	if(GAnew->intWeight(e_uOldParent) < 0){
		std::cout << "Kante: " << e_uOldParent << ", Fluss: " << GAnew->intWeight(e_uOldParent) << std::endl;
		throw std::domain_error("negatives Kantengewicht");
	}
    Gnew->delEdge(e_uOldParent);

    ogdf::edge e_uNewParent = Gnew->newEdge(newParentNode, uNode);

	int flowDiff = 1;
	for(auto c: childrenToAdopt){
		flowDiff += GAnew->intWeight(Gnew->searchEdge(newParentNode, searchNode(c, *Gnew)));
	}
    GAnew->intWeight(e_uNewParent) = flowDiff;
// 	std::cout << "in redirect Schritt Kante zu neuem Elter, Kante: " << e_uNewParent << ", Fluss: " <<  GAnew->intWeight(e_uNewParent) << std::endl;
    cost += getDistanceTo(uNode, newParentNode)  * cableCostPerUnit(flowDiff);
	
    //Flowdiff abziehen von dem alten Pfad und Kosten auf dem Pfad aktualisieren
    ogdf::node it = oldParentNode;
    while(isTurbine(it)) {
        auto p = graphalgos::parent(it, G);
        assert(p);
        p = searchNode(p->index(), *Gnew);
        ogdf::edge e = Gnew->searchEdge(p, searchNode(it->index(), *Gnew));
        if(!e) {
            throw std::domain_error("Kante existiert nicht! Sollte nicht passieren");
        }
        double distance = getDistanceTo(it, p);
        double costOlde = distance * cableCostPerUnit(GAnew->intWeight(e));
        GAnew->intWeight(e) -= 1;
// 		std::cout << "in redirectAdoption Schritt Flow abziehen, Kante: " << e << ", Fluss: " <<  GAnew->intWeight(e) << std::endl;
		if(GAnew->intWeight(e) < 0){
			std::cout << " in redirect: Kante: " << e << ", Fluss: " << GAnew->intWeight(e) << std::endl;
			throw std::domain_error("negatives Kantengewicht");
		}
        double costNewe = distance * cableCostPerUnit(GAnew->intWeight(e));
        cost = cost - costOlde + costNewe;
        assert(equalDoubleDelta(cost, costTotal(*Gnew, *GAnew)));
        if( cost <= 0) {
//             std::cout << "kosten negativ bei Flowdiff auf altem Zweig abziehen! cost: " << cost << ", costOlde " << costOlde << ", costNewe " << costNewe << std::endl;
            throw  std::domain_error("Kosten negativ nach pfad" + std::to_string(p->index()) + "->" + std::to_string(it->index())) ;
        }

        it = p;
    }

    //alle Kinder von u werden von oldParent "adoptiert", also direkt verbunden + Kosten aktualisieren
    for(auto c: childrenU) {
        if(c->index() == newParent) {
            continue;
        }
        auto eOld = Gnew->searchEdge(uNode, c);
        assert(eOld);
        int flow = GAnew->intWeight(eOld);
        double costE = getDistanceTo(uNode, c) * cableCostPerUnit(flow);
        cost -= costE;
        Gnew->delEdge(eOld);
        auto eNew = Gnew->newEdge(oldParentNode, c);
        GAnew->intWeight(eNew) = flow;
		if(GAnew->intWeight(eNew) < 0){
			std::cout << "Kante: " << eNew << ", Fluss: " << GAnew->intWeight(eNew) << std::endl;
			throw std::domain_error("negatives Kantengewicht");
		}
        cost += getDistanceTo(oldParentNode, c) * cableCostPerUnit(flow);
        if( cost <= 0) {
//             std::cout << "kosten negativ bei us Kinder von deren Großelter adoptieren! cost: " << cost << ", costE " << costE <<  std::endl;
            throw  std::domain_error("Kosten negativ nach pfad" + std::to_string(u) + "->" + std::to_string(c->index())) ;
        }
        assert(equalDoubleDelta(cost, costTotal(*Gnew, *GAnew)));
    }

    //update flow on new branch to substation + check for capacities + update cost
    it = newParentNode;
    while(isTurbine(it)) {
        ogdf::node p = graphalgos::parent(it, *Gnew);
        if(!p) {
            //std::cout << "hat kein Elter" << std::endl;
        }
        ogdf::edge e = Gnew->searchEdge(p, it);
        if(!e) {
            throw std::domain_error("Kante existiert nicht! Sollte nicht passieren");
        }
        double distance = getDistanceTo(it, p);
        double costOlde = distance * cableCostPerUnit(GAnew->intWeight(e));
        GAnew->intWeight(e) += flowDiff;
		if(GAnew->intWeight(e) < 0){
			std::cout << "Kante: " << e << ", Fluss: " << GAnew->intWeight(e) << std::endl;
			throw std::domain_error("negatives Kantengewicht");
		}
        int flowNew = GAnew->intWeight(e);
        if(isTurbine(p) && flowNew >= nS_) {
            return boost::none;
        } else if (!isTurbine(p) && flowNew >= nC_) {
            return boost::none;
        }
        cost = cost - costOlde + distance * cableCostPerUnit(flowNew);
        if( cost <= 0) {
            throw  std::domain_error("Kosten negativ nach pfad" + std::to_string(p->index()) + "->" + std::to_string(it->index()) );
        }
         assert(equalDoubleDelta(cost, costTotal(*Gnew, *GAnew)));
        it = p;
    }

    // adopt childrenToAdopt from u (delete the edges to newParent + new Edges to u)
    for(auto c: childrenToAdopt) {
        auto cNode = searchNode(c, *Gnew);
        if(c == oldParent) {
            continue;
        }
        auto eOld = Gnew->searchEdge(newParentNode, cNode);
        assert(eOld);
        int flow = GAnew->intWeight(eOld);
        cost -= getDistanceTo(newParentNode, cNode) * cableCostPerUnit(flow);
        Gnew->delEdge(eOld);
        auto eNew = Gnew->newEdge(uNode, cNode);
        GAnew->intWeight(eNew) = flow;
		if(GAnew->intWeight(eNew) < 0){
			std::cout << "Kante: " << eNew << ", Fluss: " << GAnew->intWeight(eNew) << std::endl;
			throw std::domain_error("negatives Kantengewicht");
		}
        cost += getDistanceTo(uNode, cNode) * cableCostPerUnit(flow);
         assert(equalDoubleDelta(cost, costTotal(*Gnew, *GAnew)));
        if( cost <= 0) {
            throw std::domain_error("Kosten negativ nach pfad" + std::to_string(newParent) + "->" + std::to_string(c)) ;
        }
    }
    return std::make_tuple(std::move(Gnew), std::move(GAnew), cost);
}

std::pair<std::unique_ptr<ogdf::Graph>, std::unique_ptr<ogdf::GraphAttributes>> Windfarm::buildMST(){
	auto Gnew = std::make_unique<ogdf::Graph> (topoG_);
	auto GAnew = std::make_unique<ogdf::GraphAttributes> (*Gnew,
							 ogdf::GraphAttributes::nodeId |
	                         ogdf::GraphAttributes::edgeIntWeight);
	
	
	ogdf::node root = Gnew->newNode(Gnew->numberOfNodes()+1);
	
	for(auto s: substations_){
		auto sNew = searchNode(s->index(), *Gnew);
		Gnew->newEdge(root, sNew);
	}
	
    ogdf::EdgeArray<double> weight(*Gnew);
	for(auto e: Gnew->edges){
		if(e->source() == root){
			weight[e] = 1;
		} else {
			weight[e] = getDistanceTo(e->source(), e->target());
		}
	}
	ogdf::NodeArray<ogdf::edge> pred(*Gnew);
	ogdf::EdgeArray<bool> isInTree(*Gnew);
	
	ogdf::computeMinST(root, *Gnew, weight, pred, isInTree);
	
	std::vector<ogdf::edge> toDelete;
	for(auto e: Gnew->edges){
		if(!isInTree[e]) {
			toDelete.emplace_back(e);
		}
	}
	for(auto e: toDelete){
		Gnew->delEdge(e);
	}
	graphalgos::toArborescene(*Gnew, root);
	

	toDelete.clear();
	for(auto adj: root->adjEntries){
		toDelete.emplace_back(adj->theEdge());
	}
	Gnew->delNode(root);
	
	if(!ogdf::isForest(*Gnew)){
		std::cout << "MST nach Kanten + root löschen kein Wald!" << std::endl;
	}

	//Fluss bestimmen:
	computeFlow(*Gnew, *GAnew);
	
	return make_pair(std::move(Gnew), std::move(GAnew));
}
void Windfarm::printCableLayout() {
    std::cout << "turbine -> parent, Fluss" << std::endl;
    for(auto t: turbines_) {
        auto p = graphalgos::parent(t, *cableLayout_);
        auto e = cableLayout_->searchEdge(searchNode(t->index(), *cableLayout_), searchNode(p->index(), *cableLayout_));
        if(e) {
            std::cout << t->index() << " -> " << p->index() << ", Fluss: " << cableLayoutGA_->intWeight(e) << std::endl;
        }
    }
}


double Windfarm::getDistanceTo(ogdf::node const v, ogdf::node const w) const {
    ogdf::edge e = topoG_.searchEdge((searchNode(v->index(), topoG_)), searchNode(w->index(), topoG_));
    if(e != nullptr) {
        return topoGA_.doubleWeight(e);
    } else {
        return __INT_MAX__;
    }
}

double Windfarm::getEuklideanDistance(const ogdf::node v, const ogdf::node w) const {
    ogdf::node vTopo = searchNode(v->index(), topoG_);
    ogdf::node wTopo = searchNode(w->index(), topoG_);
    ogdf::edge e = topoG_.searchEdge(vTopo, wTopo);
    if(e != nullptr) {
        return topoGA_.doubleWeight(e);
    } else {
        double vX = topoGA_.x(vTopo);
        double vY = topoGA_.y(vTopo);
        double wX = topoGA_.x(wTopo);
        double wY = topoGA_.y(wTopo);

        return std::hypot((vX-wX), (vY-wY));
    }
}

double Windfarm::getMinDistance() {
    double mindistance = 1000;
    for(auto e: topoG_.edges) {
        ogdf::node s = e->source();
        ogdf::node t = e->target();
        double d = getDistanceTo(s, t);
        if(d < mindistance) {
            mindistance = d;
        }
    }
    return mindistance;
}

ogdf::node Windfarm::getClosestSubstation(ogdf::node v) {
    ogdf::node vtopo = searchNode(v->index(), topoG_);
    double distance = __INT_MAX__;
    ogdf::node subs = substations_[0];
    for(auto s: substations_) {
        double d = getDistanceTo(v, s);
        if ( d == __INT_MAX__) {
            double sX = topoGA_.x(s);
            double sY = topoGA_.y(s);
            double vX = topoGA_.x(vtopo);
            double vY = topoGA_.y(vtopo);
            d = std::hypot((sX-vX), (sY-vY));
        }
        if(d < distance) {
            distance = d;
            subs = s;
        }

    }
    return subs;
}

void Windfarm::computeFlow(ogdf::Graph &G, ogdf::GraphAttributes& GA) {
    //initialize flow with 0
    for(auto e: G.edges) {
	    GA.intWeight(e) = 1;
    }
//      cableToSVG("ohneFluss.svg", G, GA);
    if(!ogdf::isForest(G)) {
        throw std::domain_error("cableLayout_ ist kein Wald!");
    }
    //This is the set A in the paper
    std::set<ogdf::node> turbines(turbines_.begin(), turbines_.end());

    //this is the set B in the paper
    ogdf::List<ogdf::edge> edg;
    G.allEdges(edg);
    std::set<ogdf::edge> edges(edg.begin(), edg.end());

    std::map<ogdf::node, int> flowAtNode;
    for(auto v: G.nodes) {
        if(isTurbine(v)) {
            flowAtNode.emplace(v, 1);
        } else {
            flowAtNode.emplace(v, 0);
        }
    }
    while(turbines.size() > 0) {

        ogdf::node nodeA = chooseLeaf(edges, turbines, G);
// 		nodeA = searchNode(nodeA->index(), G);
        if(!nodeA) {
            throw std::domain_error("kein node A gefunden!");
        }

        ogdf::node toNode;
        ogdf::edge cable;
        for(auto e: edges) {
			if (e->isIncident(searchNode(nodeA->index(), G))) {
                cable = e;
				toNode = e->opposite(searchNode(nodeA->index(), G));

            }
        }

		int flowA = flowAtNode.at(searchNode(nodeA->index(), G));		

        flowAtNode.at(searchNode(toNode->index(), G)) += flowA;
        GA.intWeight(cable) = flowA;
        turbines.erase(nodeA);
        edges.erase(cable);
    }
}

ogdf::node Windfarm::chooseLeaf(std::set<ogdf::edge>& edges, std::set<ogdf::node>& nodes, ogdf::Graph const& G) {

//     for(auto e: edges) {
//         std::cout << e << ", ";
//     }
//     std::cout << std::endl;

    for(auto n: nodes) {
        int edgecount = 0;
        for(auto e: edges) {
            if (e->isIncident(searchNode(n->index(), G))) {
                edgecount++;
            }
        }
//         std::cout << "chooseLeaf: node = " << n << ", edgecount = " << edgecount << ", nodes.size() = " << nodes.size() << ", edges.size() = " << edges.size() << std::endl;

        if(edgecount == 1) {
            return n;
        }
    }
    return nullptr;
}


double Windfarm::costTotal(ogdf::Graph const& G, ogdf::GraphAttributes const& GA) {
    double cost = 0;

    for(auto e: G.edges) {
        int cableCost = cableCostPerUnit(GA.intWeight(e));
        auto s = e->source();
        auto t = e->target();
        cost += cableCost * getDistanceTo(s, t);
    }
    return cost;
}

double Windfarm::realCost(){
	double cost = 0;
	
	for(auto e: cableLayout_->edges) {
		int cableCost = cableCostPerUnit(cableLayoutGA_->intWeight(e));
		auto s = e->source();
		auto t = e->target();
		double costCable = cableCost * getEuklideanDistance(s, t);
		//std::cout << "kante: " << e << ", Fluss auf Kante: " << cableLayoutGA_->intWeight(e) << ", Kosten pro Einheit: " << cableCost << ", Ksoten für ganze Kante: " << costCable << std::endl;
		cost += costCable;
	}
    return cost;
}


bool Windfarm::isTurbine(ogdf::node const v) {

    if(topoGA_.fillColor(searchNode(v->index(), topoG_)).toString()== "#6A5ACD") {
        return true;
    } else {
        return false;
    }
}



int Windfarm::cableCostPerUnit(int flow) const {

    //array of cable types sorted by capacity,
    //tuples (capacity,cost per unit),
    //cable types from Berzan et al
    constexpr std::array<std::array<int, 2>, 24> cableTypes { {{0,0},
			{5, 20},
            {8, 25},
            {12, 27},
            {15, 41},
			{30, 82},
			{45, 123},
			{60, 164},
			{75, 205},
			{90, 246},
			{105, 287},
			{120, 329},
			{135, 370},
			{150, 410},
			{165, 451},
			{180, 492},
			{195, 533},
			{210, 574},
			{225, 615},
			{240, 656},
			{255, 697},
			{270, 738},
			{285, 779},
			{300, 820}
			
        }
    };

    for(auto cable:cableTypes) {
        if (flow <= cable[0]) {
            return cable[1];
		}
    }
    return INVALID_CABLE;
}

double Windfarm::getAspectRatio(){
	double xMax = 0;
	double yMax = 0;
	for(auto v: topoG_.nodes){
		if(topoGA_.x(v) > xMax){
			xMax = topoGA_.x(v);
		}
		if(topoGA_.y(v) > yMax){
			yMax = topoGA_.y(v);
		}
	}
	return xMax/yMax;
}

void Windfarm::cableToSVG(ogdf::string filename, ogdf::Graph const& G, ogdf::GraphAttributes const& GA) const {

    ogdf::Graph outputG(G);

    ogdf::GraphAttributes outputGA(outputG,
                                   ogdf::GraphAttributes::nodeGraphics |
                                   ogdf::GraphAttributes::edgeGraphics |
                                   ogdf::GraphAttributes::nodeLabel |
                                   ogdf::GraphAttributes::edgeStyle |
                                   ogdf::GraphAttributes::nodeStyle |
                                   ogdf::GraphAttributes::nodeTemplate |
                                   ogdf::GraphAttributes::nodeId |
                                   ogdf::GraphAttributes::edgeIntWeight);

    outputGA.directed() = true;

    //Graphattribute kopieren
    for(auto v: outputG.nodes) {
        ogdf::node const origV = searchNode(v->index(), topoG_);
        outputGA.x(v) = topoGA_.x(origV);
        outputGA.y(v) = topoGA_.y(origV);
        outputGA.fillColor(v) = topoGA_.fillColor(origV);
        outputGA.width(v)= 2*topoGA_.width(origV);
        outputGA.height(v) = topoGA_.height(origV);
        outputGA.label(v) = std::to_string( origV->index());
        outputGA.shape(v) = topoGA_.shape(origV);
        outputGA.fillPattern(v) = topoGA_.fillPattern(origV);
        outputGA.strokeWidth(v) = topoGA_.strokeWidth(origV);
        if(topoGA_.fillColor(origV) =="#6A5ACD" ) {
            outputGA.fillColor(v) = "#B1A9E5";
        }
    }

    for(auto e: outputG.edges) {
        ogdf::node sourceOrig = searchNode(e->source()->index(), G);
		if(!sourceOrig){
			std::cout << e->source() << " nicht in Ursprungsgraph" << std::endl;
		}
        ogdf::node targetOrig = searchNode(e->target()->index(), G);
		if(!targetOrig){
			std::cout << e->target() << " nicht in Ursprungsgraph" << std::endl;
		}
        ogdf::edge eOrig = G.searchEdge(sourceOrig, targetOrig);
		if(!eOrig){
			std::cout << e << " nicht in Ursprungsgraph" << std::endl;
		}
		int weight = GA.intWeight(eOrig);
        outputGA.intWeight(e) = weight;
    }

    for(auto e: outputG.edges) {
        ogdf::node const sourceOrig = searchNode(e->source()->index(), topoG_);
        ogdf::node const targetOrig = searchNode(e->target()->index(), topoG_);
        double distance = getDistanceTo(sourceOrig, targetOrig);
		int weight = outputGA.intWeight(e);
// 		std::cout << "cableToSVG, Kante " << e << " betrachten, weight: " << weight << std::endl;
        if(distance == __INT_MAX__) {
            outputGA.strokeType(e) = ogdf::StrokeType( ogdf::StrokeType::Dash);
        }
        if(weight <= 5) {
            outputGA.strokeColor(e) = ogdf::Color(ogdf::Color::Name::Green);
        } else if (weight <= 8) {
            outputGA.strokeColor(e) = ogdf::Color(ogdf::Color::Name::Red);
        } else if (weight <= 12) {
            outputGA.strokeColor(e) = ogdf::Color(ogdf::Color::Name::Blue);
        } else if (weight <= 15) {
            outputGA.strokeColor(e) = ogdf::Color(ogdf::Color::Name::Orange);
		} else if (weight > 15){
			outputGA.strokeColor(e) = ogdf::Color(ogdf::Color::Name::Cyan);
		}
    }

    ogdf::GraphIO::write(outputGA, filename, ogdf::GraphIO::drawSVG);

}

bool equalDoubleDelta(double d1, double d2) {
    if(std::abs(d1-d2) < 1/10000.0) {
        return true;
    }
    return false;
}

