
#include <random>
#include <boost/range/irange.hpp>
#include <boost/range/numeric.hpp>

#include "ClusterTable.h"
#include "gen_visitations.h"
#include "clustering_preselection.h"
#include "in_cluster_distances.h"
#include "clustering_density.h"

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

std::unordered_map<size_t, double> clustering::select_cluster_members(
        const std::vector<std::pair<size_t, double>> &candidate_members,
        const size_t level) {
    double mean = 0.0;
    for (auto&[_,distance]: candidate_members) mean += distance;
    mean /= (double) candidate_members.size();

    std::unordered_map<size_t, double> selection;

    for (auto [id,distance]: candidate_members) {
        if (distance <= mean*1.5) {
            selection.insert({id, distance});
        }
    }
    return selection;
}

std::vector<Cluster> clustering::create_cluster_candidates(PointToPointPlanner &point_to_point_planner,
                                                           const std::vector<StateAtGoal> &goal_samples,
                                                           const std::vector<Cluster> &clusters,
                                                           const PreselectionStrategy &preselect,
                                                           const PostSelectionStrategy &postselect,
                                                           size_t level,
                                                           double time_per_ptp) {

    // Build a GNAT to perform large-scale NN-lookups.
    auto gnat = buildLayerGNAT(clusters);

    return boost::copy_range<std::vector<Cluster>>(boost::irange<size_t>(0, clusters.size()) | boost::adaptors::transformed([&](size_t cluster){
        // The new cluster
        auto new_cluster = Cluster::new_around(clusters, cluster);

        auto nearby_samples = preselect.select_around(cluster, gnat, level);

        std::cout << "Attempting to connect to " << nearby_samples.size() << " neighbours." << std::endl;

        std::vector<std::pair<size_t,double>> candidate_members;

        // Try to connect to nearby sub-clusters.
        for (const auto &nearby_sample: nearby_samples) {
            // The expensive part: Try to plan from representative to representative.
            if (auto ptp = point_to_point_planner.planToOmplState(time_per_ptp,
                                                                  clusters[cluster].representative->get(),
                                                                  clusters[nearby_sample].representative->get())) {
                // If successful, store this as a cluster member.
                candidate_members.emplace_back(nearby_sample, ptp->length());
            }
        }

        for (auto& [mid, dist] : postselect.select_cluster_members(candidate_members, level)) {
            new_cluster.add_reachable(clusters, mid, dist);
        }

        std::cout << "Successfully reached: " << new_cluster.members.size() << "." << std::endl;

        return new_cluster;
    }));
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


std::vector<size_t>
clustering::select_clusters(const std::vector<Cluster> &clusters, const std::vector<Cluster> &parent_layer) {

    std::cout << "Selecting from " << clusters.size() << " candidates." << std::endl;

    struct InQueue { size_t cluster_id; double density_at_insertion;

        // Queue by density first.
        bool operator<(const InQueue &rhs) const {
            return density_at_insertion < rhs.density_at_insertion;
        }
    };

    std::priority_queue<InQueue> by_density;

    std::vector<bool> cluster_visited(clusters.size(),false);
    for (auto cluster_id : boost::irange<size_t>(0,clusters.size())) {
        by_density.push({cluster_id, cluster_filtered_density(cluster_visited, clusters[cluster_id], parent_layer)});
    }

    std::vector<size_t> new_clusters;

    // Keep going until all has been processed.
    while (!by_density.empty()) {

        // Take the remaining cluster with the highest density.
        // This is a global maximum, hence also a local maximum.
        InQueue current_cluster = by_density.top();
        by_density.pop();

        if (cluster_visited[current_cluster.cluster_id]) {
            // Do nothing, discard this cluster.
        } else {
            // Re-compute the density to check whether it has changed.
            double density = cluster_filtered_density(cluster_visited,
                                                      clusters[current_cluster.cluster_id],
                                                      parent_layer);

            if (density == current_cluster.density_at_insertion) {
                // No change! We can proceed.

                cluster_visited[current_cluster.cluster_id] = true;
                std::cout << "Accepted:" << current_cluster.cluster_id << " density: " << current_cluster.density_at_insertion << std::endl;

                new_clusters.push_back(current_cluster.cluster_id);

                for (const auto &member: clusters[current_cluster.cluster_id].members) {
                    cluster_visited[member.first] = true;
                }

            } else {
                // It has! Put it back in the queue...
                // This will happen at most once per member.
                by_density.push({current_cluster.cluster_id, density});
            }
        }
    }

    std::cout << "Selected: " << new_clusters.size() << std::endl;

    return new_clusters;
}

double clustering::cluster_filtered_density(const std::vector<bool> &cluster_visited, const Cluster &cluster,
                                            const std::vector<Cluster> &parent_layer) {
    return boost::accumulate(cluster.members | boost::adaptors::transformed([&](const auto& pair){
                return cluster_visited[pair.first] ? 0.0 : ((double)parent_layer[pair.first].goals_reachable.size() / pair.second);
            }), 0.0);
}

std::vector<std::vector<Cluster>>
clustering::buildClusters(PointToPointPlanner &point_to_point_planner, const std::vector<StateAtGoal> &goal_samples,
                          const PreselectionStrategy &preselect, const PostSelectionStrategy &postselect,
                          const DensityStrategy &densityStrategy) {

    // Start by building singleton clusters: one for every goal sample.
    std::vector<std::vector<Cluster>> cluster_hierarchy = {
            buildTrivialClusters(goal_samples)
    };

    size_t iters = 0;

    while (cluster_hierarchy.back().size() > 1 && iters < 500) {

        // Expand the clusters to connect them to neighbouring clusters
        auto new_clusters = create_cluster_candidates(point_to_point_planner,
                                                      goal_samples,
                                                      cluster_hierarchy.back(),
                                                      preselect,
                                                      postselect,
                                                      iters,
                                                      0.2);

        auto densities = densityStrategy.computeForLayer(new_clusters, cluster_hierarchy.back());

        auto selection = select_clusters(new_clusters, cluster_hierarchy.back());

        if (selection.size() < cluster_hierarchy.back().size()) {
            cluster_hierarchy.emplace_back(/*empty*/);

            for (const auto &item: selection) {
                cluster_hierarchy.back().push_back(new_clusters[item]);
            }
        }

        iters += 1;
    }

    std::reverse(cluster_hierarchy.begin(), cluster_hierarchy.end());

    if (cluster_hierarchy.front().size() > 1) {
        ROS_WARN("Cluster hierarchy is not connected.");
    }

    return cluster_hierarchy;
}

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
