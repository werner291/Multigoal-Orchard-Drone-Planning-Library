
#include "ClusterTable.h"

ClusterTable::ClusterTable(const GoalSet &goals) {

}

struct StateAtGoal {
    size_t goal_idx;
    ompl::base::ScopedStatePtr state;
};

struct InCluster {
    size_t member_id;
    double path_distance_from_center;
};

struct ClusterNeighbour {
    size_t through_member;
    size_t to_cluster;
};

struct Cluster {
    size_t representative;
    std::vector<InCluster> members;
    std::vector<ClusterNeighbour> neighbours;
};

std::vector<double> computeDensities(std::vector<Cluster> &new_clusters);

std::vector<StateAtGoal>
takeInitialSamples(const GoalSet &goals, const ompl::base::SpaceInformationPtr &si, const int samples_per_goal) {

    std::vector<StateAtGoal> goal_samples;

    for (size_t goal_idx = 0; goal_idx < goals.size(); ++goal_idx) {
        for (size_t i = 0; i < samples_per_goal; ++i) {
            ompl::base::ScopedStatePtr goal_sample(new ompl::base::ScopedState(si));
            goals[goal_idx]->sampleGoal(goal_sample->get());
            goal_samples.push_back({goal_idx, goal_sample});
        }
    }

    return goal_samples;
}

ompl::NearestNeighborsGNAT<size_t> buildGoalSampleGnat(const std::vector<StateAtGoal> &goal_samples) {

    ompl::NearestNeighborsGNAT<size_t> gnat;

    gnat.setDistanceFunction([&](const size_t &a, const size_t &b) {
        return goal_samples[a].state->distance(goal_samples[b].state->get());
    });

    for (size_t i = 0; i < goal_samples.size(); i++) { gnat.add(i); }

    return gnat;
}

std::vector<std::vector<size_t>> find_overlap(const std::vector<Cluster> &clusters) {
    std::vector<std::vector<size_t>> goal_in_clusters(clusters.size());

    for (const auto &cluster: clusters) {
        for (const auto &in_cluster: cluster.members) {
            goal_in_clusters[in_cluster.member_id].push_back(cluster.representative);
        }
    }

    return goal_in_clusters;
}

void expandClusters(PointToPointPlanner &point_to_point_planner,
                    const std::vector<StateAtGoal> &goal_samples,
                    double threshold,
                    std::vector<Cluster> &clusters) {

    ompl::NearestNeighborsGNAT<size_t> gnat;
    gnat.setDistanceFunction([&](const size_t &a, const size_t &b) {
        return goal_samples[a].state->distance(goal_samples[b].state->get());
    });

    for (const Cluster &cluster: clusters) {
        gnat.add(cluster.representative);
    }

    for (Cluster &cluster: clusters) {

        std::vector<size_t> nearby_samples;
        gnat.nearestR(cluster.representative, threshold, nearby_samples);

        for (const auto &nearby_sample: nearby_samples) {
            auto ptp = point_to_point_planner.planToOmplState(0.2,
                                                              goal_samples[cluster.representative].state->get(),
                                                              goal_samples[nearby_sample].state->get());

            if (ptp) { // TODO avoid duplicates here
                cluster.members.push_back({nearby_sample, ptp->length()});
            }
        }
    }
}

std::vector<Cluster> buildTrivialClusters(GoalSet &goals, const std::vector<StateAtGoal> &goal_samples) {
    std::vector<Cluster> clusters(goals.size());

    for (size_t sample_id = 0; sample_id < goal_samples.size(); ++sample_id) {
        clusters[sample_id].representative = sample_id;
    }

    return clusters;
}

std::vector<double> computeDensities(const std::vector<Cluster> &new_clusters) {
    std::vector<double> densities;
    densities.reserve(new_clusters.size());

    for (const auto &cluster: new_clusters) {
        double density = 0.0;
        for (const auto &item: cluster.members) {
            density += 1.0 / item.path_distance_from_center;
        }
        densities.push_back(density);
    }
    return densities;
}

std::vector<size_t> findDensityMaxima(const std::vector<Cluster> &new_clusters, const std::vector<double> &densities) {

    assert(new_clusters.size() == densities.size());

    std::vector<size_t> maxima;

    for (size_t i = 0; i < new_clusters.size(); ++i) {
        if (std::all_of(new_clusters[i].members.begin(), new_clusters[i].members.end(), [&](const InCluster &member) {
            return densities[member.member_id] <= densities[i];
        }))
            maxima.push_back(i);
    }

    return maxima;

}

std::vector<Cluster> recluster(PointToPointPlanner &point_to_point_planner,
                               const std::vector<StateAtGoal> &goal_samples,
                               double threshold,
                               const std::vector<Cluster> &sub_clusters) {

    std::vector<Cluster> candidate_clusters = sub_clusters;

    // Expand the clusters to connect them to neighbouring clusters
    expandClusters(point_to_point_planner, goal_samples, threshold, candidate_clusters);

    auto densities = computeDensities(candidate_clusters);
    auto maxima = findDensityMaxima(candidate_clusters, densities);

    std::vector<Cluster> new_clusters;

    for (const auto maximum: maxima) {
        new_clusters.push_back(std::move(candidate_clusters[maximum]));
    }

    return new_clusters;
}


std::vector<std::vector<Cluster>> buildClusters(GoalSet &goals,
                                                PointToPointPlanner &point_to_point_planner,
                                                const std::vector<StateAtGoal> &goal_samples) {
    double threshold = 0.1;
    double growFactor = 1.5;
    double maxDistance = 100.0;

    // Start by building singleton clusters: one for every goal sample.
    // TODO: Idea: don't copy the clusters this much, just build a hierarchy of references.
    std::vector<std::vector<Cluster>> cluster_hierarchy = {buildTrivialClusters(goals, goal_samples)};

    while (threshold < maxDistance) {
        threshold *= growFactor;

        std::vector<Cluster> new_clusters = cluster_hierarchy.back();

        // Expand the clusters to connect them to neighbouring clusters
        expandClusters(point_to_point_planner, goal_samples, threshold, new_clusters);

        auto overlap = find_overlap(new_clusters);

        cluster_hierarchy.emplace_back(/*empty*/);

        for (size_t i = 0; i < new_clusters.size(); ++i) {
            if (overlap[i].size() < 2) {
                // TODO: Idea: use this as a way to link clusters together.
                // TODO: Incorporate planning distances somehow.

                cluster_hierarchy.back().push_back(std::move(new_clusters[i]));
            }
        }
    }

    return cluster_hierarchy;
}
//
//void heap_recurse(size_t depth,
//                  const Eigen::Vector3d &start_position,
//                  std::vector<std::pair<size_t, Eigen::Vector3d>> &target_positions,
//                  const double costSoFar,
//                  double &bestCost,
//                  std::vector<std::pair<size_t, Eigen::Vector3d>> &bestSolution) {
//
//    // Cost can only go up as we go deeper, so no point in doing so
//    // if the prefix is more expensive than the best-known solution.
//    if (costSoFar > bestCost) return;
//
//    if (depth == target_positions.size()) {
//
//        // The order of the whole array is fixed, the cost of this order is thus fully known.
//        // Record it if it is better than the best-known.
//        if (costSoFar < bestCost) {
//            bestCost = costSoFar;
//            bestSolution = target_positions;
//        }
//
//    } else {
//
//        // The point from which we travel to the next candidate goal.
//        Eigen::Vector3d previous = depth == 0 ? start_position : target_positions[depth - 1].second;
//
//        // Go through all unvisited goals as possible candidates as the next to visit.
//        for (size_t i = depth; depth < target_positions.size(); ++i) {
//
//            // Swap it into the next position.
//            std::swap(target_positions[i], target_positions[depth]);
//
//            // Recurse, with that position now fixed.
//            heap_recurse(depth + 1, start_position, target_positions,
//                         costSoFar + (previous - target_positions[depth].second).norm(), bestCost, bestSolution);
//
//            // Undo the swap to make sure the candidate selection process is not disturbed.
//            std::swap(target_positions[i], target_positions[depth]);
//
//        }
//    }
//}

MultiGoalPlanResult ClusterBasedPlanner::plan(GoalSet &goals, const ompl::base::State *start_state,
                                              PointToPointPlanner &point_to_point_planner,
                                              std::chrono::milliseconds time_budget) {

    auto goal_samples = takeInitialSamples(goals, point_to_point_planner.getPlanner()->getSpaceInformation(), 50);
    auto gnat = buildGoalSampleGnat(goal_samples);
    auto clusters = buildClusters(goals, point_to_point_planner, goal_samples);

    // Use a branch-and-bound strategy that is guided using the cluster hierarchy to rapidly lower the bound as much as possible.



    return {};

}


std::string ClusterBasedPlanner::getName() {
    return "ClusterBasedPlanner";
}
