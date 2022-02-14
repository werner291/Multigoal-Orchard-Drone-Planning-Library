
#include "agglomerative_clustering.h"
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/irange.hpp>

agglomerative_clustering::AgglomerativeClustering::AgglomerativeClustering(
        const std::vector<ompl::base::ScopedStatePtr> &toCluster, const PointToPointPlanner &ptp)
        : next_node_id(0), ptp(ptp) {
    _nodes = boost::copy_range<std::vector<TreeNode>>(
            toCluster | boost::adaptors::transformed([&](const ompl::base::ScopedStatePtr &state) {
                return std::make_pair(next_node_id++,std::make_shared<TreeNode>(state, {}));
            })
    );
}

bool agglomerative_clustering::AgglomerativeClustering::iterate() {

    assert(_nodes.size() >= 2);

    merge_pair(CandidatePair());

    return _nodes.size() == 1;

}

void agglomerative_clustering::AgglomerativeClustering::merge_pair(CandidatePair pair) {
    assert(pair.is_tight());



    size_t new_node_id = next_node_id++;

    for (size_t pivot_i : boost::irange<size_t>(0,_pivots.size())) {
        if (auto path = ptp.planToOmplState(0.2,pair.midpoint->get(), _pivots[pivot_i]->get())) {
            pivot_distances[pivot_i][new_node_id] = path->length();
        } else {
            pivot_distances[pivot_i][new_node_id] = INFINITY;
        }
    }

    for (const auto& [node_id, node] : _nodes) {

        double distance = INFINITY;
        for (size_t pivot_i : boost::irange<size_t>(0,_pivots.size())) {
            distance = std::min(distance, pivot_distances[pivot_i][new_node_id]+pivot_distances[pivot_i][node_id]);
        }

        _candidate_pairs.push({
                                      {node_id, new_node_id},
                                      distance,
                                      {}
        });
    }

    _nodes[new_node_id] = { pair.midpoint, {{_nodes[pair.pair.first],_nodes[pair.pair.second]}} };
    _nodes.erase(pair.pair.first);
    _nodes.erase(pair.pair.second);
}

agglomerative_clustering::AgglomerativeClustering::CandidatePair
agglomerative_clustering::AgglomerativeClustering::extract_next_valid_candidate() {

    CandidatePair candidate;

    do {
        candidate = _candidate_pairs.top();
        _candidate_pairs.pop();
    } while (isNotClosed(candidate));

    return candidate;
}

bool agglomerative_clustering::AgglomerativeClustering::isNotClosed(
        const agglomerative_clustering::AgglomerativeClustering::CandidatePair &candidate) const {
    return _nodes.count(candidate.pair.first) && _nodes.count(candidate.pair.second);
}

agglomerative_clustering::AgglomerativeClustering::CandidatePair
agglomerative_clustering::AgglomerativeClustering::find_closest_pair() {
    auto candidate = extract_next_valid_candidate();

    while (!candidate.is_tight()) {

        tighten_and_enqueue(candidate.pair.first, candidate.pair.second);

        candidate = extract_next_valid_candidate();
    }

    return candidate;
}

void agglomerative_clustering::AgglomerativeClustering::tighten_and_enqueue(size_t a, size_t b) {
    if (auto path = ptp.planToOmplState(0.2, _nodes[a].representative, _nodes[b].representative)) {
        auto midpoint = std::make_shared<ompl::base::ScopedState<ompl::base::StateSpace>>(
                ptp.getPlanner()->getSpaceInformation());
        ptp->getPlanner()->getSpaceInformation()->copyState(midpoint->get(), path->getState(path->getNumStates() / 2));
        _candidate_pairs.emplace({a, b}, ptp->length(), midpoint);
    } else {
        ROS_WARN("Cannot plan between candidate pair.");
    }
}

void agglomerative_clustering::AgglomerativeClustering::pick_pivots(size_t n) {
    std::vector<size_t> choices;
    choices.reserve(n);

    ompl::RNG rng;

    for (size_t i: boost::irange(0, n)) {

        size_t choice = rng.uniformInt(0, _nodes.size() - choices.size());

        for (size_t j: choices) {
            if (choice >= j) {
                choice += 1;
            }
        }

        choices.push_back(choice);
    }

    _pivots = boost::copy_range<>(choices | boost::adaptors::transformed([&](size_t node_id) {
        assert(_nodes.count(node_id) == 1);
        return _nodes[node_id].representative;
    }));
}

void agglomerative_clustering::AgglomerativeClustering::computeInitialEstimates(size_t k_pivots) {

    pick_pivots(k_pivots);



    for (const auto&[node_i, _]: _nodes) {
        for (const auto&[node_j, _]: _nodes) {
            if (node_j >= node_i) continue;
            double distance_bound = INFINITY;
            for (size_t pivot_i: boost:irange(0, k_pivots)) {
                distance_bound = std::min(distance_bound,
                                          pivot_distances[pivot_i][node_i] + pivot_distances[pivot_j][node_i]);
            }
            _candidate_pairs.push({
                                         {node_i, node_j},
                                         distance_bound,
                                         {}
                                 });
        }
    }


}

void agglomerative_clustering::AgglomerativeClustering::compute_pivot_distance_matrix() {
    for (size_t pivot_i: boost:irange(0, k_pivots)) {
        auto pivot = _pivots[pivot_i];

        for (const auto&[node_id, node]: _nodes) {
            if (auto path = ptp.planToOmplState(0.2, pivot->get(), node.represenattive->get())) {
                pivot_distances[pivot_i][node_id] = path->length();
            } else {
                pivot_distances[pivot_i][node_id] = INFINITY;
            }
        }
    }
}


agglomerative_clustering::AgglomerativeClustering::TreeNode::TreeNode(const ompl::base::ScopedStatePtr &representative,
                                                                      const std::optional<std::pair<std::shared_ptr<TreeNode>, std::shared_ptr<TreeNode>>> &children)
        : representative(representative), children(children) {}
