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

#ifndef WINDFARMMOVER_H
#define WINDFARMMOVER_H

#include <ogdf/module/ClustererModule.h>
#include <ogdf/cluster/ClusterGraph.h>
#include <functional>

#include "windfarm.h"
#include "graphalgos.h"
#include "subsets.h"

constexpr int DEFAULTSTOP = 100;

using graphalgos::searchNode;
using ClusterResult = std::tuple<std::unique_ptr<ogdf::Graph>, std::unique_ptr<ogdf::GraphAttributes>, double, ogdf::SList<ogdf::SimpleCluster*>>;
using ClusterFunc = std::function<ClusterResult()>;

class WindfarmMover
{
public:
    WindfarmMover(Windfarm& wf, int stop = DEFAULTSTOP);

    ~WindfarmMover() {
        for (auto c: cluster_)
            delete c;
    }

    double averageCIndex() /*override*/;

    ClusterResult computeClustering() const;

    //wie computeclustering, aber falls sich nix verbessert, einzelne Knoten bewegen und Kinder von Großeltern adoptieren adoptieren
    ClusterResult singleMove();

    ClusterResult adoptionMove();

    void findBest(ClusterFunc func);

private:

    ogdf::SList<ogdf::SimpleCluster*> initializeCluster() const;

    Windfarm& wf_;
    ogdf::SList<ogdf::SimpleCluster*> cluster_;
    double cost_total_;
    int stop_;
    ogdf::ClusterGraph C_;

};

#endif // WINDFARMMOVER_H
