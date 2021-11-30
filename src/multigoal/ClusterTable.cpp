
#include "ClusterTable.h"

ClusterTable::ClusterTable(const GoalSet &goals) {

}

struct StateAtGoal {
    size_t goal_idx;
    ompl::base::ScopedStatePtr state;
};

struct InCluster {
    size_t goal_sample_id;
    double path_distance_from_center;
};

struct Cluster {
    size_t cluster_around;
    std::vector<InCluster> in_cluster;
};

std::vector<StateAtGoal> takeInitialSamples(const GoalSet &goals, const ompl::base::SpaceInformationPtr &si) {

    std::vector<StateAtGoal> goal_samples;

    for (size_t goal_idx = 0; goal_idx < goals.size(); ++goal_idx) {
        for (size_t i = 0; i < 50; ++i) {
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
        for (const auto &in_cluster: cluster.in_cluster) {
            goal_in_clusters[in_cluster.goal_sample_id].push_back(cluster.cluster_around);
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
        gnat.add(cluster.cluster_around);
    }

    for (Cluster &cluster: clusters) {

        std::vector<size_t> nearby_samples;
        gnat.nearestR(cluster.cluster_around, threshold, nearby_samples);

        for (const auto &nearby_sample: nearby_samples) {
            auto ptp = point_to_point_planner.planToOmplState(0.2,
                                                              goal_samples[cluster.cluster_around].state->get(),
                                                              goal_samples[nearby_sample].state->get());

            if (ptp) { // TODO avoid duplicates here
                cluster.in_cluster.push_back({nearby_sample, ptp->length()});
            }
        }
    }
}

std::vector<Cluster> buildTrivialClusters(GoalSet &goals, const std::vector<StateAtGoal> &goal_samples) const {
    std::vector<Cluster> clusters(goals.size());

    for (size_t sample_id = 0; sample_id < goal_samples.size(); ++sample_id) {
        clusters[sample_id].cluster_around = sample_id;
    }
    return clusters;
}

MultiGoalPlanResult ClusterBasedPlanner::plan(GoalSet &goals, const ompl::base::State *start_state,
                                              PointToPointPlanner &point_to_point_planner,
                                              std::chrono::milliseconds time_budget) {

    auto goal_samples = takeInitialSamples(goals, point_to_point_planner.getPlanner()->getSpaceInformation());
    auto gnat = buildGoalSampleGnat(goal_samples);

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
                // TODO: Incorproate planning distances somehow.

                cluster_hierarchy.back().push_back(std::move(new_clusters[i]));
            }
        }
    }

    // TODO Use the clusters to find a path through the goal samples.




    return {};

}


std::string ClusterBasedPlanner::getName() {
    return "ClusterBasedPlanner";
}
