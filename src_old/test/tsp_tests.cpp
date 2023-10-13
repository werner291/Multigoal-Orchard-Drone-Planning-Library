
#include <gtest/gtest.h>
#include <range/v3/all.hpp>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_parameters.h>

TEST(TSPTEST, grouped_tsp) {

    using namespace std;
    using namespace ranges;

    const double destinations[] = {
            1.0,1.5, 2.5,5.0, 5.5
    };

    auto distance_matrix = destinations
            | views::transform([&](const double &d) {
               return  destinations | views::transform([d](const double &d2) {
                    return (int64_t) (1000.0 * abs(d-d2));
                }) | to_vector;
            }) | to_vector;



    operations_research::RoutingIndexManager manager(5, 1, operations_research::RoutingIndexManager::NodeIndex {(int) 2 });

    // Build a routing model, and register the distance matrix.
    operations_research::RoutingModel routing(manager);
    routing.SetArcCostEvaluatorOfAllVehicles(routing.RegisterTransitMatrix(distance_matrix));

    routing.AddDisjunction({0,1});
//    routing.AddDisjunction({3,4});

    operations_research::RoutingSearchParameters searchParameters = operations_research::DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);

//
    const operations_research::Assignment* solution = routing.SolveWithParameters(searchParameters);



    for (int64_t index = routing.Start(0); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
        std::cout << "index: " << index << std::endl;
    }


}
