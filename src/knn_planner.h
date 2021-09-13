
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <robowflex_ompl/ompl_interface.h>

robowflex::Trajectory visitAllKnn(const std::vector<Apple> &apples,
                                  const moveit::core::RobotState &start_state,
                                  const size_t k,
                                  const robowflex::SceneConstPtr &scene,
                                  const robowflex::RobotConstPtr &robot);