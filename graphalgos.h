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

#ifndef GRAPHALGOS_H
#define GRAPHALGOS_H

#include <set>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/simple_graph_alg.h>
#include <ogdf/module/ClustererModule.h>
#include <ogdf/cluster/ClusterGraph.h>

namespace graphalgos {
bool toArborescene(ogdf::Graph& G, ogdf::node root);

std::vector<ogdf::node> children(ogdf::node v, ogdf::Graph const& G);

ogdf::node parent(ogdf::node const v, ogdf::Graph const& G);

ogdf::node searchNode(int id, ogdf::Graph const& G);

ogdf::SimpleCluster* searchCluster(int id, ogdf::SList<ogdf::SimpleCluster*> cluster);

ogdf::Graph subGraph(std::set<ogdf::node> nodes, ogdf::Graph const& G);

ogdf::SimpleCluster* getRootCluster(int index, ogdf::SList< ogdf::SimpleCluster* >const& cluster);

ogdf::cluster getRootCluster(int index, ogdf::ClusterGraph C);

int getNumberOfRootCluster(ogdf::Graph G, ogdf::SList<ogdf::SimpleCluster*> cluster);

void mergeClusters(ogdf::SimpleCluster& c1, ogdf::SimpleCluster& c2, ogdf::SList<ogdf::SimpleCluster*>& cluster);

void mergeClusters(ogdf::SList<ogdf::SimpleCluster*> toMerge, ogdf::SList<ogdf::SimpleCluster*>& cluster);

//traverses all children to get the number of all children in the cluster
int clusterSize(ogdf::SimpleCluster* c);

//deletes the first occurence of a cluster from a List of clusters, if the cluster is in the List. ogdf has nothing good for this
void delCluster(ogdf::SimpleCluster* c, ogdf::SList<ogdf::SimpleCluster*> cluster);
}
#endif // GRAPHALGOS_H
