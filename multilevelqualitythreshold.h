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

//based on the paper "A Clustering based Wind Farm Collector System Cable Layout Design" by S. Dutta, Student Member, IEEE, and T. J. Overbye, Fellow, IEEE

#ifndef MULTILEVELQUALITYTHRESHOLD_H
#define MULTILEVELQUALITYTHRESHOLD_H

#include <ogdf/basic/extended_graph_alg.h>
#include <ogdf/cluster/ClusterGraph.h>

#include "windfarm.h"
#include "graphalgos.h"
 

constexpr u_int MINSIZE = 4;
using graphalgos::searchNode;

class MultiLevelQualityThreshold
{
public:
	MultiLevelQualityThreshold(Windfarm& wf);
	
	std::set<std::set<ogdf::node>> qualityThresholdClusteringDistance(std::set<ogdf::node>& sites, double threshold, u_int minSize);
	
	std::set<std::set<ogdf::node>> qualityThresholdClusteringCost(std::map<ogdf::node, int>& sites, double threshold, u_int minSize);
	
	//Clusters only based on the distance between the turbines
	std::pair<std::unique_ptr<ogdf::Graph>, double> multiLevelClusteringDistance(double initialMultiplyer);
	
	//Considers the size of the clusters and therefore the cablecosts
	std::pair<std::unique_ptr<ogdf::Graph>, double> multiLevelClusteringCost(double initialMultiplyer);
	
	void findBestInitialThresholdDistance();
	
	void findBestInitialThresholdCost();
	
	ogdf::node findNearestValidNode(ogdf::node v, ogdf::SList<ogdf::SimpleCluster *> cluster, ogdf::Graph& G);
	
	Windfarm& wf_;
	std::set<std::set<ogdf::node>> cluster_;
};


#endif // MULTILEVELQUALITYTHRESHOLD_H
