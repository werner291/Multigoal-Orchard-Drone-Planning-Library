
#include "json_utils.h"
#include "multigoal/multi_goal_planners.h"
#include "multigoal/PointToPointPlanner.h"
#include <robowflex_library/trajectory.h>

Json::Value eigenToJson(const Eigen::Vector3d &vec) {
    Json::Value apple;
    apple[0] = vec.x();
    apple[1] = vec.y();
    apple[2] = vec.z();
    return apple;
}

Json::Value
makePointToPointJson(const std::optional<PointToPointPlanResult> &pointToPointPlanResult) {
    Json::Value json;
    json["solved"] = pointToPointPlanResult.has_value();
    if (pointToPointPlanResult.has_value()) {
        json["apple"] = eigenToJson(pointToPointPlanResult.value().endEffectorTarget);
        json["path_length"] = pointToPointPlanResult.value().solution_length;
    }
    return json;
}

Json::Value getStateStatisticsPoint(const moveit::core::RobotState &st) {
    Json::Value traj_pt;
    for (int i = 0; i < st.getVariableCount(); i += 1) {
        traj_pt["values"][i] = st.getVariablePosition(i);
    }

    return traj_pt;
}
