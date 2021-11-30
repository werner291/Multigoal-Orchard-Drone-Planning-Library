
#include "approach_table.h"
#include "../genbattle_dkm.hpp"

/// \brief Compute the GoalApproachTable with at most k valid samples per goal.
multigoal::GoalApproachTable
takeRepresentativeGoalSamples(const ompl::base::SpaceInformationPtr &si, const multigoal::SampleableGoals &goals,
                              int k) {

    multigoal::GoalApproachTable gat = multigoal::takeGoalSamples(si, goals, 1000);


    std::random_device rd;
    std::mt19937 gen(rd());

    for (auto &goal_samples: gat) {

        std::vector<size_t> medoids;

        for (size_t i = 0; i < k; ++i) {

            // TODO: Try out k-medoids++ if this doesn't work very well.

            size_t picked = std::uniform_int_distribution<size_t>(0, gat.size() - medoids.size())(gen);

            for (const auto &item: medoids) {
                if (picked >= item) picked += 1;
            }

            medoids.push_back(i);
        }


        for (size_t i = 0; i < gat.size(); i++) {

        }

        for (size_t i = 0; i < 100; ++i) {

        }

    }

//    dkm::

    return gat;

}