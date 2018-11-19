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

#ifndef WINDFARM_H
#define WINDFARM_H

#include <vector>
#include <map>
#include <stdexcept>
#include <memory>

#include <boost/optional.hpp>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/fileformats/GraphIO.h>
#include <ogdf/basic/graphics.h>
#include <ogdf/basic/List.h>
#include <ogdf/basic/extended_graph_alg.h>

#include "graphalgos.h"


constexpr int DEFAULT_NS = 42;
constexpr int DEFAULT_NC = 15;
constexpr int INVALID_CABLE = __INT_MAX__;

using graphalgos::searchNode;
using RedirectResult = boost::optional<std::tuple<std::unique_ptr<ogdf::Graph>, std::unique_ptr<ogdf::GraphAttributes>, double>>;
class Windfarm
{
public:
    Windfarm(std::string filename, int nS = DEFAULT_NS, int nC = DEFAULT_NC);
    
	bool isTurbine(ogdf::node const v);

//     ogdf::node searchNode(int id, ogdf::Graph const& G) const {
//         for(auto v: G.nodes) {
//             if(v->index() == id) {
//                 return v;
//             }
//         }
//         return nullptr;
//     }

    ogdf::Graph const& getGraph() {
        return topoG_;
    }

    void printCableLayout();
    int cableCostPerUnit(int flow) const ;

//     ogdf::node parent(ogdf::node const v, ogdf::Graph const& G);
//     ogdf::node parent(ogdf::node const v) {
//         return graphalgos::parent(v, *cableLayout_);
//     }

//     std::vector<ogdf::node> children(ogdf::node v, ogdf::Graph const& G);
    std::vector<ogdf::node> children(ogdf::node v){
		return graphalgos::children(v, *cableLayout_);
	}

    //arguments are the indices of the nodes
    RedirectResult redirect(int oldParent, int u, int newParent, double costOld, ogdf::Graph const& G, ogdf::GraphAttributes const& GA);
    
	RedirectResult redirectSingleNode(int oldParent, int u, int newParent, double costOld, ogdf::Graph const& G, ogdf::GraphAttributes const& GA);
    
	RedirectResult redirectAdoption(int oldParent, int u, int newParent, std::set<int> childrenToAdopt, double costOld, ogdf::Graph const& G, ogdf::GraphAttributes const& GA);
	
	double getDistanceTo(ogdf::node const v, ogdf::node const w) const;
	double getEuklideanDistance(ogdf::node const v, ogdf::node const w) const;
    double getAspectRatio();
	double getMinDistance();
	
	//actually gets the closest substation, even if there is no potential edge
	ogdf::node getClosestSubstation(ogdf::node v);
	
	double realCost();
	double costTotal(ogdf::Graph const& G, ogdf::GraphAttributes const& GA);
    
	void cableToSVG(std::string filename, ogdf::Graph const& G, ogdf::GraphAttributes const& GA) const;
	void cableToSVG(std::string filename) const {
		cableToSVG(filename, *cableLayout_, *cableLayoutGA_);
	}

	//based on the paper Optimal Wind Farm Collector System Topology Design Considering Total Trenching Length, needed for the algorithms by this author
	void computeFlow(ogdf::Graph& G, ogdf::GraphAttributes& GA);
	void computeFlow(){
		cableLayoutGA_->init(*cableLayout_, ogdf::GraphAttributes::nodeId | ogdf::GraphAttributes::edgeIntWeight);
		computeFlow(*cableLayout_, *cableLayoutGA_);
	}
	
	//returns a node from the set nodes, that is only incident to one edge in the set edges. nullptr if no such node exists
	ogdf::node chooseLeaf(std::set<ogdf::edge>& edges, std::set<ogdf::node>& nodes, ogdf::Graph const& G);
	
	std::pair<std::unique_ptr<ogdf::Graph>, std::unique_ptr<ogdf::GraphAttributes>> buildMST();
	
	std::pair<std::unique_ptr<ogdf::Graph>, std::unique_ptr<ogdf::GraphAttributes>> initializeCableLayout();
	std::pair<std::unique_ptr<ogdf::Graph>, std::unique_ptr<ogdf::GraphAttributes>> initializeCableLayoutDistance();
	
	
    int nS_;
    int nC_;
    std::vector<ogdf::node> turbines_;
	std::vector<ogdf::node> substations_;
    std::map<ogdf::edge, int> flow_;
    std::unique_ptr<ogdf::Graph> cableLayout_;
    std::unique_ptr<ogdf::GraphAttributes> cableLayoutGA_;
    ogdf::Graph topoG_;
    ogdf::GraphAttributes topoGA_;
};
	bool equalDoubleDelta(double d1, double d2);
#endif // WINDFARM_H
