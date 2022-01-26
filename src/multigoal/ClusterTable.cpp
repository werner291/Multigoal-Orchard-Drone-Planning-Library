
#include <random>
#include <boost/range/irange.hpp>

#include "ClusterTable.h"

using namespace clustering;

std::vector<StateAtGoal> clustering::takeInitialSamples(const GoalSet &goals, const ompl::base::SpaceInformationPtr &si,
                                                        const int samples_per_goal) {

    std::vector<StateAtGoal> goal_samples;

    for (size_t goal_idx = 0; goal_idx < goals.size(); ++goal_idx) {
        for (size_t i = 0; i < samples_per_goal; ++i) {
            ompl::base::ScopedStatePtr goal_sample(new ompl::base::ScopedState(si));
            goals[goal_idx]->sampleGoal(goal_sample->get());
            if (si->isValid(goal_sample->get())) {
                goal_samples.push_back({goal_idx, goal_sample});
            }
        }
    }

    return goal_samples;
}

/**
 * Build a GNAT of StateAteGoal, using state distance as the distance metric.
 *
 * The resulting datastructure refers into the provided `goal_samples` vector,
 * and does not copy its' contents.
 */
ompl::NearestNeighborsGNAT<size_t> buildGoalSampleGnat(const std::vector<StateAtGoal> &goal_samples) {

    ompl::NearestNeighborsGNAT<size_t> gnat;

    gnat.setDistanceFunction([&](const size_t &a, const size_t &b) {
        return goal_samples[a].state->distance(goal_samples[b].state->get());
    });

    for (size_t i = 0; i < goal_samples.size(); i++) {
        gnat.add(i);
    }

    return gnat;
}

ompl::NearestNeighborsGNAT<size_t> buildLayerGNAT(const std::vector<Cluster> &clusters) {
    // Build a GNAT to perform large-scale NN-lookups.
    ompl::NearestNeighborsGNAT<size_t> gnat;
    gnat.setDistanceFunction([&](const size_t &a, const size_t &b) {
        return clusters[a].representative->distance(clusters[b].representative->get());
    });
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) gnat.add(cluster);
    return gnat;
}

std::vector<size_t> search_r_try_n(const ompl::NearestNeighborsGNAT<size_t>& gnat,
                                   size_t around,
                                   double radius,
                                   size_t n) {
    std::vector<size_t> nearby_samples;
    gnat.nearestR(around, radius, nearby_samples);
    truncate(nearby_samples, n);
    return nearby_samples;
}

std::vector<size_t> nearestK(const ompl::NearestNeighborsGNAT<size_t>& gnat,
                                   size_t around,
                                   size_t n) {
    std::vector<size_t> nearby_samples;
    gnat.nearestK(around, n, nearby_samples);
    return nearby_samples;
}

std::vector<Cluster> clustering::create_cluster_candidates(PointToPointPlanner &point_to_point_planner,
                                                           const std::vector<StateAtGoal> &goal_samples,
                                                           double threshold,
                                                           const std::vector<Cluster> &clusters,
                                                           const size_t max_cluster_size,
                                                           double time_per_ptp) {

    // Build a GNAT to perform large-scale NN-lookups.
    auto gnat = buildLayerGNAT(clusters);

    // We keep a list of new clusters to return.
    std::vector<Cluster> new_clusters;

    // There is one candidate for every sub-cluster in the previous layer.
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) {

        // The new cluster
        auto new_cluster = Cluster::new_around(clusters, cluster);

        // Find which sub-clusters are near this sub-cluster by checking distance to the representative.
        auto nearby_samples = nearestK(gnat, cluster, max_cluster_size);
//         auto nearby_samples = search_r_try_n(gnat, cluster, threshold, max_cluster_size);

        std::cout << "Attempting to connect to " << nearby_samples.size() << " neighbours." << std::endl;

        size_t successes = 0;;

        // Try to connect to nearby sub-clusters.
        for (const auto &nearby_sample: nearby_samples) {

            // Filter out the cluster itself.
            if (nearby_sample == cluster) { continue;
//             else if (nearby_sample < cluster) {
//                 // We've already seen this one, so just look up if we can connect to it.
//                 auto itr = new_clusters[nearby_sample].members.find(cluster);
//                 if (itr != new_clusters[nearby_sample].members.end()) {
//                     // Yes, a path has already been planned to this representative, so just look it up.
//                     // As a bonus, this makes the connections symmetric.
//                     new_cluster.add_reachable(clusters, nearby_sample, itr->second);
//                     successes++;
//                 } else { // Else, it was probably unreachable.
//                     std::cout << "Cache lookup failed." << std::endl;
//                 }
            } else {
                // The expensive part: Try to plan from representative to representative.
                auto ptp = point_to_point_planner.planToOmplState(time_per_ptp,
                                                                  clusters[cluster].representative->get(),
                                                                  clusters[nearby_sample].representative->get());

                std::cout << "Attempting to connect to number " << nearby_sample << std::endl;

                // If successful, store this as a cluster member.
                if (ptp) {
                    new_cluster.add_reachable(clusters, nearby_sample, ptp->length());
                    std::cout << "Connected to number " << nearby_sample << std::endl;

                } else {
                    std::cout << "planning failed" << std::endl;
                }
            }
        }

        std::cout << "planning succeeded to:";
        for (auto m:new_cluster.members) {
            std::cout << m.first << ",";
        }
        std::cout << std::endl;


        new_clusters.push_back(new_cluster);
    }

    return new_clusters;
}

std::vector<Cluster> clustering::buildTrivialClusters(const std::vector<StateAtGoal> &goal_samples) {
    std::vector<Cluster> clusters(goal_samples.size());

    for (size_t sample_id = 0; sample_id < goal_samples.size(); ++sample_id) {
        clusters[sample_id].representative = goal_samples[sample_id].state;
        clusters[sample_id].members[sample_id] = 0.0;
        clusters[sample_id].goals_reachable = {goal_samples[sample_id].goal_idx};
    }

    return clusters;
}

 std::vector<std::vector<DistanceMatrix>> clustering::computeAllDistances(PointToPointPlanner &point_to_point_planner, const ClusterHierarchy& clusters) {

     assert(clusters.size() >= 1);

        std::vector<std::vector<DistanceMatrix>> distances(clusters.size()-1);

        for (size_t layer_i = 0; layer_i + 1 < clusters.size(); ++layer_i) {

            std::cout << "Computing distances for layer: " << layer_i << std::endl;

            size_t i = 0;

            distances[layer_i] = boost::copy_range<std::vector<DistanceMatrix> >(
                clusters[layer_i] | boost::adaptors::transformed([&](const Cluster& cl) {

                std::cout << "Cluster : " << (i++) << std::endl;

                return computeDistanceMatrix(point_to_point_planner, cl, clusters[layer_i+1]);
            }));
        }

        return distances;
    }

std::vector<double> clustering::computeDensities(const std::vector<Cluster> &new_clusters) {
    std::vector<double> densities;
    densities.reserve(new_clusters.size());

    for (const auto &cluster: new_clusters) {
        double density = 0.0;
        for (const auto &item: cluster.members) {
            if (item.second > 0.0) { density += 1.0 / item.second; }
        }
        densities.push_back(density);
    }
    return densities;
}

std::vector<size_t> clustering::select_clusters(const std::vector<Cluster> &clusters,
                                                std::vector<double> densities) {

    std::cout << "Selecting from " << clusters.size() << " candidates." << std::endl;

//     for (auto cl : clusters) {
//         std::cout << "Cluster with members: ";
//         for (auto m:cl.members) {
//             std::cout << m.first << ",";
//         }
//         std::cout << std::endl;
//     }

    struct InQueue {
        size_t cluster_id;
        double density_at_insertion;
    };

    // Build a priority queue that returns the cluster index with highest density first.
    auto cmp = [&](const InQueue &a, const InQueue &b) {
        return a.density_at_insertion < b.density_at_insertion;
    };

    std::priority_queue<InQueue, std::vector<InQueue>, decltype(cmp)> by_density(cmp);
    std::vector<bool> cluster_visited(densities.size());

    for (size_t i = 0; i < densities.size(); ++i) {
        by_density.push({i, densities[i]});
        cluster_visited[i] = false;
    }

    std::vector<size_t> new_clusters;

    // Keep going until all has been processed.
    while (!by_density.empty()) {

        // Take the remaining cluster with the highest density.
        // This is a global maximum, hence also a local maximum.
        InQueue current_cluster = by_density.top();

//         std::cout << "Looking at cluster " << current_cluster.cluster_id << std::endl;

        // If the cluster has been visited, it is already a part of one of the new clusters.
        // If the density at insertion doesn't match, perform lazy deletion since we'll be
        // running into this cluster again later in the correct order.
        if (cluster_visited[current_cluster.cluster_id] ||
            densities[current_cluster.cluster_id] != current_cluster.density_at_insertion) {
            // Do nothing.

            if (densities[current_cluster.cluster_id] != current_cluster.density_at_insertion) {
//                 std::cout << "Rejected by lazy-deletion." << std::endl;
            } else {
//                 std::cout << "Pruned a cluster." << std::endl;
            }


        } else {
            assert(!cluster_visited[current_cluster.cluster_id]);
            cluster_visited[current_cluster.cluster_id] = true;
//             std::cout << "Accepted:" << current_cluster.cluster_id << std::endl;

            new_clusters.push_back(current_cluster.cluster_id);

            for (const auto &member: clusters[current_cluster.cluster_id].members) {
                cluster_visited[member.first] = true;
//                 if (member.first != current_cluster.cluster_id)
//                     std::cout << "Marked a member cluster as visited:" << member.first << std::endl;
                // FIXME Should update the SECOND ORDER densities
//                 densities[member.first] -= 1.0 / member.second;
//                 by_density.push({member.first, densities[member.first]});
            }
        }

        by_density.pop();
    }

    std::cout << "Selected: " << new_clusters.size() << std::endl;

    return new_clusters;
}


std::vector<std::vector<Cluster>>
clustering::buildClusters(PointToPointPlanner &point_to_point_planner, const std::vector<StateAtGoal> &goal_samples) {

    double threshold = 0.1;
    double growFactor = 1.2;

    // Start by building singleton clusters: one for every goal sample.
    // TODO: Idea: don't copy the clusters this much, just build a hierarchy of references.
    std::vector<std::vector<Cluster>> cluster_hierarchy = {buildTrivialClusters(goal_samples)};

    size_t max_iters = 500;

    while (cluster_hierarchy.back().size() > 1 && --max_iters > 0) {
        threshold *= growFactor;

        std::cout << "Threshold: " << threshold << " Top: " << cluster_hierarchy.back().size() << std::endl;

        // Expand the clusters to connect them to neighbouring clusters
        auto new_clusters = create_cluster_candidates(point_to_point_planner, goal_samples, threshold,
                                                      cluster_hierarchy.back());

        auto densities = computeDensities(new_clusters);

        auto selection = select_clusters(new_clusters, densities);

        if (selection.size() < cluster_hierarchy.back().size()) {
            cluster_hierarchy.emplace_back(/*empty*/);

            for (const auto &item: selection) {
                cluster_hierarchy.back().push_back(new_clusters[item]);
            }
        }
    }

    std::reverse(cluster_hierarchy.begin(), cluster_hierarchy.end());

    if (cluster_hierarchy.front().size() > 1) {
        ROS_WARN("Cluster hierarchy is not connected.");
    }

    return cluster_hierarchy;
}

std::vector<size_t>
clustering::visit_clusters_naive(const std::vector<std::vector<Cluster>> &cluster_hierarchy,
                                 const std::vector<StateAtGoal> &goal_samples,
                                 const ompl::base::State *start_state,
                                 const ompl::base::SpaceInformation &si) {

    std::vector<size_t> visit_order;

    struct StackFrame {
        size_t level;
        size_t in_level;
    };

    std::set<size_t> goals_visited;

    std::vector<StackFrame> stack;
    for (size_t i = 0; i < cluster_hierarchy.back().size(); ++i) {
        stack.push_back({cluster_hierarchy.size() - 1,
                         i
                        });
    }

    while (!stack.empty()) {

        StackFrame frame = stack.back();
        stack.pop_back();

        const Cluster &cluster = cluster_hierarchy[frame.level][frame.in_level];

        std::vector<size_t> additional_reachable_goals;

        std::set_difference(
                cluster.goals_reachable.begin(), cluster.goals_reachable.end(),
                goals_visited.begin(), goals_visited.end(),
                std::back_inserter(additional_reachable_goals)
        );

        if (!additional_reachable_goals.empty()) {

            if (frame.level == 0) {
                assert(additional_reachable_goals.size() == 1);
                visit_order.push_back(frame.in_level);
                goals_visited.insert(additional_reachable_goals.begin(), additional_reachable_goals.end());
            } else {
                for (const auto &item: cluster.members) {
                    stack.push_back({
                                            frame.level - 1,
                                            item.first
                                    });
                }
            }
        }
    }

    return visit_order;
};

DistanceMatrix clustering::computeDistanceMatrix(
    PointToPointPlanner &point_to_point_planner,
    const Cluster& cluster,
    const std::vector<Cluster> &parent_clusters,
    double planningTimePerPair) {

    // For convenience, we copy over the parent cluster indices of the members.
    std::vector<size_t> state_ids;
    state_ids.reserve(cluster.members.size());
    for (const auto& m : cluster.members) {
        assert(m.first < parent_clusters.size());
        state_ids.push_back(m.first);
    }

    clustering::DistanceMatrix output;

    for (size_t i : boost::irange<size_t>(0,state_ids.size())) {
        output[std::make_pair(state_ids[i],state_ids[i])] = 0.0;
        for (size_t j = 0; j < i; ++j) {

            auto ptp = point_to_point_planner.planToOmplState(
                planningTimePerPair,
                parent_clusters[state_ids[i]].representative->get(),
                parent_clusters[state_ids[j]].representative->get());

            output[std::make_pair(state_ids[i],state_ids[j])] = ptp ? ptp->length() : INFINITY;
            output[std::make_pair(state_ids[j],state_ids[i])] = ptp ? ptp->length() : INFINITY;
            std::cout << "Planning from " << state_ids[i] << " to " << state_ids[i];
        }
    }

    return output;

}

std::vector<size_t> determine_visitation_order(const ompl::base::ScopedStatePtr &start_pos,
                                               const ClusterHierarchy &hierarchy,
                                               const ClusterDistanceFn &distanceFn) {

    assert(hierarchy[0].size() == 1);

    // Base case: hierarchy is guaranteed to have exactly one cluster at the top.
    std::vector<clustering::Visitation> previous_layer_order = {
            {0, hierarchy[0][0].goals_reachable}
    };

    // Step case: go layer-by-layer down the hierarchy.
    for (size_t layer_i = 1; layer_i < hierarchy.size(); ++layer_i) {

        std::cout << "Ordering on layer " << layer_i << std::endl;

        // Build up a visitation order.
        std::vector<clustering::Visitation> layer_order;

        // Get a reference to the current layer of the hierarchy...
        const std::vector<Cluster> &layer = hierarchy[layer_i];
        // And the lower LOD layer that we just came through.
        const std::vector<Cluster> &previous_layer = hierarchy[layer_i - 1];

        // In the previous step, we determined an order in which to visit the clusters.
        // We will now do so, in that order.
        for (size_t visit_i = 0; visit_i < previous_layer_order.size(); ++visit_i) {

            // We're looking for the optimal order in which to traverse the cluster members.
            clustering::VisitationOrderSolution sub_layer_best_solution{
                    {}, INFINITY, {}
            };

            // The ordering is aware of entry-and-exit-points.
            // For the first in the sequence, that's the starting position of the robot.
            auto entry_point = start_pos;
            if (visit_i > 0) {
                // On subsequent sub-clusters, we use the previous cluster's representative as a reference point.
                entry_point = layer[previous_layer_order[visit_i - 1].subcluster_index].representative;
            }

            // Same for the exit point, except that we don't have a specified end point for the global sequence,
            // so it's only relevant if we know we'll be visiting another cluster after this.
            std::optional<ompl::base::ScopedStatePtr> exit_point;
            if (visit_i + 1 < previous_layer_order.size()) {
                exit_point = layer[previous_layer_order[visit_i + 1].subcluster_index].representative;
            }

            // Get the cluster members and put them in a vector.
            auto members_vec = boost::copy_range<std::vector<size_t>>(
                    previous_layer[previous_layer_order[visit_i].subcluster_index].members
                    | boost::adaptors::map_keys);

            // Distance function. Either looks up the representative of a cluster based on index,
            // or uses the given reference point directly.
            auto distance = [&](const StateOrClusterRef &a, const StateOrClusterRef &b) {
                return distanceFn.distance(a, b, layer_i, previous_layer_order[visit_i].subcluster_index);
            };

            // Look up a cluster's reachable goal identifiers.
            // Note the explicit return type: if not, the set is de-referenced, then a reference
            // is returned to that temporary copy, causing a use-after-free bug.
            auto lookup_reachable = [&](const size_t &a) -> const std::set<size_t> & {
                return layer[a].goals_reachable;
            };

            // Go through every possible permutation of the clusters and keep track of the best.
            clustering::generate_visitations<ompl::base::ScopedStatePtr, size_t>(
                    entry_point, members_vec, previous_layer_order[visit_i].goals_to_visit, exit_point,
                    distance,
                    lookup_reachable,
                    [&](auto soln) {
                        if (soln.is_better_than(sub_layer_best_solution)) {
                            sub_layer_best_solution = soln;
                        }
                    });

            // Concatenate to the total solution for this abstraction layer.
            for (const auto &visit: sub_layer_best_solution.visit_order) {
                layer_order.push_back({
                                              // Note: generate_visitations produces a vector of indices into the vector given to it,
                                              // so we need to perform a lookup to translate to cluster layer indices.
                                              members_vec[visit.subcluster_index],
                                              visit.goals_to_visit
                                      });
            }
        }

        previous_layer_order = layer_order;
    }

    // Strip the goal visitation information and just return the indices as a vector.
    return boost::copy_range<std::vector<size_t>>(
            previous_layer_order | boost::adaptors::transformed([](const auto &elt) {
                return elt.subcluster_index;
            }));
}

//
//std::vector<size_t>
//    clustering::visit_clusters(const std::vector<std::vector<Cluster>> &cluster_hierarchy,
//                           const std::vector<StateAtGoal> &goal_samples,
//                           const ompl::base::State* start_state,
//                           const ompl::base::SpaceInformation& si) {
//
//    std::set<size_t> goals_reachable;
//    for (const auto &cluster : cluster_hierarchy[0])
//        goals_reachable.insert(cluster.goals_reachable.begin(),
//                               cluster.goals_reachable.end());
//
//    std::vector<size_t> visit_order = index_vector(cluster_hierarchy[0]);
//
//    std::random_device rand;
//    std::mt19937 rng(rand());
//
//    std::shuffle(visit_order.begin(), visit_order.end(), rng);
//
//    size_t visit_n = visit_order.size() - std::min(std::geometric_distribution<size_t>(0.5)(rng),visit_order.size());
//
//    double cost_estimate = si.distance(start_state, goal_samples[cluster_hierarchy[0][visit_order.front()].representative_sample_id].state->get());
//
//    std::set<size_t> goals_visited;
//
//    for (size_t visit_i = 1; visit_i < visit_n; visit_i++) {
//        goals_visited.insert(cluster_hierarchy[0][visit_order[visit_i]].goals_reachable.begin(),
//                             cluster_hierarchy[0][visit_order[visit_i]].goals_reachable.end());
//    }
//
//    for (size_t visit_i = 1; visit_i < visit_n; visit_i++) {
//
//        // TODO: This'll do for now and hopefully give some results already, but perhaps we can actually compute
//        //       this distance since we have relatively few states. It might actually already have been planned
//        //       during the clustering stage; can we be smarter about this?
//
//        cost_estimate += goal_samples[cluster_hierarchy[0][visit_i - 1].representative_sample_id].state->distance(
//                goal_samples[cluster_hierarchy[0][visit_i].representative_sample_id].state->get()
//                );
//
//    }
//
//    double cost_per_target = cost_estimate / (double) goals_visited.size();
//    double proportion_visited = (double) goals_visited.size() / (double) goals_reachable.size();
//    double solution_badness = cost_per_target * (1.0/proportion_visited);
//
//}

MultiGoalPlanResult ClusterBasedPlanner::plan(const GoalSet &goals, const ompl::base::State *start_state,
                                              PointToPointPlanner &point_to_point_planner,
                                              std::chrono::milliseconds time_budget) {

    auto goal_samples = takeInitialSamples(goals, point_to_point_planner.getPlanner()->getSpaceInformation(), 50);
    auto gnat = buildGoalSampleGnat(goal_samples);
    auto clusters = buildClusters(point_to_point_planner, goal_samples);

    // Use a branch-and-bound strategy that is guided using the cluster hierarchy to rapidly lower the bound as much as possible.



    return {};

}


std::string ClusterBasedPlanner::getName() {
    return "ClusterBasedPlanner";
}

bool clustering::VisitationOrderSolution::is_better_than(const VisitationOrderSolution &sln) const {
    return goals_visited.size() > sln.goals_visited.size() ||
           (goals_visited.size() == sln.goals_visited.size() && cost < sln.cost);
}

void Cluster::add_reachable(const std::vector<Cluster> &parent_layer, size_t which, double distance) {
    members[which] = distance;
    goals_reachable.insert(parent_layer[which].goals_reachable.begin(),
                           parent_layer[which].goals_reachable.end());
}

Cluster Cluster::new_around(const std::vector<Cluster> &parent_layer, size_t parent) {
    return {parent_layer[parent].representative, {{parent, 0.0}}, parent_layer[parent].goals_reachable};
}

StraightDistanceMetric::StraightDistanceMetric(const ClusterHierarchy &clusters) : clusters(clusters) {}

double StraightDistanceMetric::distance(const StateOrClusterRef &a,
                                        const StateOrClusterRef &b,
                                        size_t layer_id,
                                        size_t cluster_id) const  {
    const auto layer = clusters[layer_id];

    const auto repr_a = std::holds_alternative<ompl::base::ScopedStatePtr>(a) ? std::get<ompl::base::ScopedStatePtr>(a) : layer[std::get<size_t>(
            a)].representative;

    const auto repr_b = std::holds_alternative<ompl::base::ScopedStatePtr>(b) ? std::get<ompl::base::ScopedStatePtr>(b) : layer[std::get<size_t>(
            b)].representative;

    return repr_a->distance(repr_b->get());
}

InClusterPrecomputedDistanceMetricWithFallback::InClusterPrecomputedDistanceMetricWithFallback(
        const StraightDistanceMetric &fallback,
        const std::vector<std::vector<DistanceMatrix>>& distanceMatrix)
        : fallback(fallback), distanceMatrices(distanceMatrix) {}

InClusterPrecomputedDistanceMetricWithFallback::InClusterPrecomputedDistanceMetricWithFallback(
        const ClusterHierarchy& clusters,
        const std::vector<std::vector<DistanceMatrix>>& distanceMatrix)
        : fallback(clusters), distanceMatrices(distanceMatrix) {}

double InClusterPrecomputedDistanceMetricWithFallback::distance(const StateOrClusterRef &a,
                                                                const StateOrClusterRef &b,
                                                                size_t layer_id,
                                                                size_t cluster_id) const {

    if (std::holds_alternative<ompl::base::ScopedStatePtr>(a) || std::holds_alternative<ompl::base::ScopedStatePtr>(b)) {
        return fallback.distance(a, b, layer_id, cluster_id);
    } else {
        auto key = std::make_pair(std::get<size_t>(a),std::get<size_t>(b));

        auto entry = distanceMatrices[layer_id-1][cluster_id].find(key);

        assert(entry != distanceMatrices[layer_id][cluster_id].end());

        return entry->second;
    }
}
