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

#include "graphalgos.h"
namespace graphalgos {
bool toArborescene(ogdf::Graph& G, ogdf::node root) {
    if(G.numberOfEdges() == 0) {
        return true;
    }
    if(!ogdf::isTree(G)) {
        return false;
    }

    std::set<ogdf::node> toWork;
    std::set<ogdf::node> worked;
    toWork.insert(root);


    while(toWork.size() >0) {
        for(auto v: toWork) {

            for(auto adj: v->adjEntries) {
                ogdf::edge e = adj->theEdge();

                ogdf::node t = e->opposite(v);
                toWork.insert(t);
                if(e->source()!=v && worked.find(t) == worked.end()) {
                    G.reverseEdge(e);
                }

            }
            worked.insert(v);
        }
        for(auto v: worked) {
            toWork.erase(v);
        }
    }
    return ogdf::isArborescence(G, root);
}

std::vector<ogdf::node> children(ogdf::node v, const ogdf::Graph& G)
{
    std::vector<ogdf::node> children;
    auto vG = searchNode(v->index(), G);
    for(auto const& adj: vG->adjEntries) {
        ogdf::edge e = adj->theEdge();
        if(e->source() == vG) {
            children.emplace_back(e->target());
        }
    }
    return children;
}

ogdf::node parent(const ogdf::node v, const ogdf::Graph& G) {
    auto vNode = searchNode(v->index(), G);
    for(auto const& adj: vNode->adjEntries) {
        ogdf::edge e = adj->theEdge();
        if(e->target() == vNode) {
            return e->source();
        }
    }
    return nullptr;
}

ogdf::node searchNode(int id, ogdf::Graph const& G) {
    for(auto v: G.nodes) {
        if(v->index() == id) {
            return v;
        }
    }
    return nullptr;
}

ogdf::SimpleCluster* searchCluster(int id, ogdf::SList<ogdf::SimpleCluster *> cluster) {
    for(auto c: cluster) {
        if(c->getIndex() == id) {
            return c;
        }
    }
    return nullptr;
}

ogdf::Graph subGraph(std::set<ogdf::node> nodes, ogdf::Graph const& G) {
//     for(auto v: nodes) {
//         assert(v->graphOf() == &G);
//     }
    ogdf::Graph subgraph;
    for(auto v: nodes) {
        subgraph.newNode(v->index());
    }
    for(auto v: nodes) {
        for(auto adj: v->adjEntries) {
            ogdf::node t = adj->theEdge()->opposite(v);
            auto search = nodes.find(t);
            if(search != nodes.end()) {
                subgraph.newEdge(searchNode(v->index(), subgraph), searchNode(t->index(), subgraph));
            }
        }
    }
    return subgraph;
}

int getNumberOfRootCluster(ogdf::Graph G, ogdf::SList<ogdf::SimpleCluster*> cluster) {
    std::vector<int> rootID;
    for(auto v: G.nodes) {
        int root = getRootCluster(v->index(), cluster)->getIndex();
        if(std::find(std::begin(rootID), std::end(rootID), root) == std::end(rootID)) {
            rootID.emplace_back(root);
        }
    }
    return rootID.size();
}

void mergeClusters(ogdf::SimpleCluster& c1, ogdf::SimpleCluster& c2, ogdf::SList<ogdf::SimpleCluster*>& cluster) {
    cluster.emplaceBack(new ogdf::SimpleCluster(nullptr));
    auto parent = cluster.back();
    parent->pushBackChild(&c1);
    parent->pushBackChild(&c2);
    parent->setIndex(cluster.size());

    c1.setParent(parent);
    c2.setParent(parent);
}

void mergeClusters(ogdf::SList<ogdf::SimpleCluster *> toMerge, ogdf::SList<ogdf::SimpleCluster *>& cluster) {
    cluster.emplaceBack(new ogdf::SimpleCluster(nullptr));
    auto parent = cluster.back();
    parent->setIndex(cluster.size());
    for(auto c: toMerge) {
        parent->pushBackChild(c);
		for(auto v: c->nodes()){
			parent->pushBackVertex(v);
		}
        c->setParent(parent);
    }
//     std::cout << "parent.getIndex() = " << parent->getIndex() << std::endl;
}


ogdf::SimpleCluster * getRootCluster(int index, ogdf::SList<ogdf::SimpleCluster*> const& cluster) {
    ogdf::SimpleCluster* clust = searchCluster(index, cluster);
    while(clust->getParent()!=0) {
        clust = clust->getParent();
    }
    return clust;
}

ogdf::cluster getRootCluster(int index, ogdf::ClusterGraph C) {
    ogdf::cluster clust;
    bool found = false;

    for (auto c : C.clusters) {
        if ( c->index() == index) {
            clust = c;
            found = true;
            break;
        }
    }
    if(found == false) {
        throw std::domain_error( "node not found");
    }
    while(clust->parent()!= nullptr) {
        clust = clust->parent();
    }
    return clust;
}

int clusterSize(ogdf::SimpleCluster* c){

	ogdf::SList<ogdf::SimpleCluster *> toWork = c->children();
	ogdf::SList<ogdf::SimpleCluster *> worked({c});
	
	while(toWork.size() > 0){
		ogdf::SList<ogdf::SimpleCluster *> foundCluster;
		for(auto it: toWork){
			ogdf::SList<ogdf::SimpleCluster *>& kids = it->children();
			for(auto k: kids){
				if(worked.search(k) != worked.end()){
					foundCluster.pushBack(k);
				}
			}
			worked.pushBack(it);
		}
		toWork = foundCluster;
	}
// 	std::cout << "bei clustersize gefundene Cluster:" << std::endl;
// 	for( auto x: worked){
// 		
// 	}
	return worked.size();
}

void delCluster(ogdf::SimpleCluster* c, ogdf::SList<ogdf::SimpleCluster *> cluster){
	if(cluster.search(c) != cluster.end()){
		ogdf::SListIterator<ogdf::SimpleCluster*> cBefore;
		for(auto it: cluster){
			if(cluster.cyclicSucc(cluster.search(it)) == cluster.search(c)){
				cluster.search(it) = cBefore;
				break;
			}
		}
		if(cBefore == cluster.begin()){
			cluster.popFront();
		} else {
			cluster.delSucc(cBefore);
		}
	}
}

}
