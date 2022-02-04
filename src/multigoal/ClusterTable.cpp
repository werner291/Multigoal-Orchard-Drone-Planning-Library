
#include <random>
#include <boost/range/irange.hpp>

#include "ClusterTable.h"
#include "gen_visitations.h"
#include "clustering_preselection.h"
#include "in_cluster_distances.h"

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

std::vector<Cluster> clustering::create_cluster_candidates(PointToPointPlanner &point_to_point_planner,
                                                           const std::vector<StateAtGoal> &goal_samples,
                                                           const std::vector<Cluster> &clusters,
                                                           const PreselectionStrategy &preselect,
                                                           double time_per_ptp) {

    // Build a GNAT to perform large-scale NN-lookups.
    auto gnat = buildLayerGNAT(clusters);

    // We keep a list of new clusters to return.
    std::vector<Cluster> new_clusters;

    // There is one candidate for every sub-cluster in the previous layer.
    for (size_t cluster = 0; cluster < clusters.size(); cluster++) {

        // The new cluster
        auto new_cluster = Cluster::new_around(clusters, cluster);

        auto nearby_samples = preselect.select_around(cluster, gnat, 0);

        std::cout << "Attempting to connect to " << nearby_samples.size() << " neighbours." << std::endl;

        std::vector<std::pair<size_t,double>> candidate_members;

        // Try to connect to nearby sub-clusters.
        for (const auto &nearby_sample: nearby_samples) {

            // Filter out the cluster itself.
            if (nearby_sample == cluster) {
                continue;
            } else {
                // The expensive part: Try to plan from representative to representative.
                auto ptp = point_to_point_planner.planToOmplState(time_per_ptp,
                                                                  clusters[cluster].representative->get(),
                                                                  clusters[nearby_sample].representative->get());

                // If successful, store this as a cluster member.
                if (ptp) {
                    candidate_members.emplace_back(nearby_sample, ptp->length());
                }
            }
        }


        double mean = 0.0;
        for (auto&[_,distance]: candidate_members) mean += distance;
        mean /= candidate_members.size();

        for (auto&[id,distance]: candidate_members) {
            if (distance <= mean*1.5) {
                new_cluster.add_reachable(clusters, id, distance);
            }
        }

        std::cout << "Accepted " << new_cluster.members.size() << " out of " << candidate_members.size() << std::endl;

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

std::vector<std::vector<Cluster>> clustering::buildClusters(PointToPointPlanner &point_to_point_planner,
                                                            const std::vector<StateAtGoal> &goal_samples,
                                                            const PreselectionStrategy& preselect) {

    double threshold = 0.1;
    double growFactor = 1.2;

    // Start by building singleton clusters: one for every goal sample.
    std::vector<std::vector<Cluster>> cluster_hierarchy = {buildTrivialClusters(goal_samples)};

    size_t max_iters = 500;

    while (cluster_hierarchy.back().size() > 1 && --max_iters > 0) {
        threshold *= growFactor;

        std::cout << "Threshold: " << threshold << " Top: " << cluster_hierarchy.back().size() << std::endl;

        // Expand the clusters to connect them to neighbouring clusters
        auto new_clusters = create_cluster_candidates(point_to_point_planner,
                                                      goal_samples,
                                                      cluster_hierarchy.back(),
                                                      preselect);

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
//    auto clusters = buildClusters(point_to_point_planner, goal_samples);

    // Use a branch-and-bound strategy that is guided using the cluster hierarchy to rapidly lower the bound as much as possible.


    return {};

}


std::string ClusterBasedPlanner::getName() {
    return "ClusterBasedPlanner";
}

void Cluster::add_reachable(const std::vector<Cluster> &parent_layer, size_t which, double distance) {
    members[which] = distance;
    goals_reachable.insert(parent_layer[which].goals_reachable.begin(),
                           parent_layer[which].goals_reachable.end());
}

Cluster Cluster::new_around(const std::vector<Cluster> &parent_layer, size_t parent) {
    return {parent_layer[parent].representative, {{parent, 0.0}}, parent_layer[parent].goals_reachable};
}