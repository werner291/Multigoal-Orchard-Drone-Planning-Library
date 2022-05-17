#include "SphereShell.h"

#include <utility>
#include <range/v3/all.hpp>

SphereShell::SphereShell(Eigen::Vector3d center, double radius) : center(std::move(center)), radius(radius) {

}

moveit::core::RobotState SphereShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) const {
    moveit::core::RobotState st(drone);

    Eigen::Vector3d default_facing(0.0, 1.0, 0.0);
    Eigen::Vector3d required_facing = (center - a.center).normalized();
    Eigen::Vector3d base_facing = (Eigen::Vector3d(required_facing.x(), required_facing.y(), 0.0)).normalized();

    double yaw = copysign(acos(default_facing.dot(base_facing)), default_facing.cross(base_facing).z());

    Eigen::Quaterniond qd(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    st.setVariablePositions({
                                    0.0, 0.0, 0.0,      // Position off the side of the tree
                                    qd.x(), qd.y(), qd.z(), qd.w(),// Identity rotation
                                    0.0, 0.0, 0.0, 0.0  // Arm straight out
                            });

    st.update(true);

    Eigen::Vector3d apple_on_sphere = applePositionOnShell(a);
//    if (apple_on_sphere.z() < 0.5) apple_on_sphere.z() = 0.5; // FIXME This is problematic... Need to guarantee minimum distance from the tree AND the ground.

    Eigen::Vector3d offset = (apple_on_sphere) - st.getGlobalLinkTransform("end_effector").translation();

    st.setVariablePosition(0, offset.x());
    st.setVariablePosition(1, offset.y());
    st.setVariablePosition(2, offset.z());

    st.update(true);

    return st;
}

Eigen::Vector3d SphereShell::applePositionOnShell(const Apple &a) const {
    return (a.center - center).normalized() * radius + center;
}

std::vector<moveit::core::RobotState> SphereShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) const {

    Eigen::Vector3d ra_ray = a.center - center;
    Eigen::Vector3d rb_ray = b.center - center;

    Eigen::Vector3d normal = ra_ray.cross(rb_ray).normalized();
    double angle_cos = ra_ray.dot(rb_ray) / (ra_ray.norm() * rb_ray.norm());
    double angle = acos(std::clamp(angle_cos, -1.0, 1.0)); // Rounding errors sometimes cause the cosine to be slightly outside the valid range.
    assert(!isnan(angle));
    const auto num_states = (size_t) (10.0 * angle) + 1;

    std::vector<moveit::core::RobotState> path;
    path.reserve(num_states + 1);

    std::optional<Eigen::Vector3d> offset_vector {};

    for (size_t state_i = 0; state_i <= num_states; state_i++) {

        double t = (double) (state_i) / (double) (num_states);

        Eigen::Vector3d base_local = Eigen::AngleAxisd(angle * t, normal) * ra_ray.normalized() * radius;

        if (abs(base_local.z())/radius > 0.98) {
            double tt = (abs(base_local.z())/radius - 0.98)/0.02;

            if (!offset_vector) {
                offset_vector = { normal.topRows<2>().dot(base_local.topRows<2>()) > 0.0 ? (-normal) : normal };
            }

            base_local -= tt * *offset_vector;
            //base_local.z() -= tt*normal.z();
        }

        Eigen::Vector3d base_center = center + base_local;

        path.push_back(this->state_on_shell(drone, {base_center, {0.0, 0.0, 0.0}}));
    }

    return path;

}

const Eigen::Vector3d &SphereShell::getCenter() const {
    return center;
}

double SphereShell::getRadius() const {
    return radius;
}

const Eigen::Vector2d &CylinderShell::getCenter() const {
    return center;
}

double CylinderShell::getRadius() const {
    return radius;
}

[[nodiscard]] moveit::core::RobotState CylinderShell::state_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a) const {

    Eigen::Vector2d required_facing = (center - a.center.topRows<2>()).normalized();

    double yaw = -atan2(required_facing.y(), required_facing.x());// - M_PI_2 / 2.0;

    Eigen::Quaterniond qd(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    moveit::core::RobotState st(drone);
    st.setVariablePositions({
                                    0.0, 0.0, 0.0,      // Position off the side of the tree
                                    qd.x(), qd.y(), qd.z(), qd.w(),
                                    0.0, 0.0, 0.0, 0.0  // Arm straight out
                            });

    st.update(true);

    Eigen::Vector3d apple_on_sphere = applePositionOnShell(a);
    Eigen::Vector3d offset = (apple_on_sphere) - st.getGlobalLinkTransform("end_effector").translation();

    st.setVariablePosition(0, offset.x());
    st.setVariablePosition(1, offset.y());
    st.setVariablePosition(2, offset.z());

    st.update(true);

    return st;
}

[[nodiscard]] std::vector<moveit::core::RobotState> CylinderShell::path_on_shell(const moveit::core::RobotModelConstPtr &drone, const Apple &a, const Apple &b) const {

    Eigen::Vector3d
        ra_ray(a.center.x() - center.x(), a.center.y() - center.y(), 0.0),
        rb_ray(b.center.x() - center.x(), b.center.y() - center.y(), 0.0);

    Eigen::Vector3d normal = ra_ray.cross(rb_ray).normalized();
    double angle_cos = ra_ray.dot(rb_ray) / (ra_ray.norm() * rb_ray.norm());
    double angle = acos(std::clamp(angle_cos, -1.0, 1.0)); // Rounding errors sometimes cause the cosine to be slightly outside the valid range.

    using namespace ranges;

    return views::linear_distribute(0.0, 1.0, (long) (5.0 * angle) + 1)
        | views::transform([&](double t) {
        Eigen::Vector3d on_cylinder(
                center.x() + radius * cos(t * M_PI * 2.0),
                center.y() + radius * sin(t * M_PI * 2.0),
                a.center.z() * (1.0 - t) + b.center.z() * t
        );
        return state_on_shell(drone, {on_cylinder, {0.0, 0.0, 0.0}});
    }) | to_vector;
}

[[nodiscard]] Eigen::Vector3d CylinderShell::applePositionOnShell(const Apple &a) const {

    Eigen::Vector2d flat_heading = (a.center.topRows<2>() - center).normalized() * radius;

    return {
        flat_heading.x(),
        flat_heading.y(),
        a.center.z()
    };
}

CylinderShell::CylinderShell(const Eigen::Vector2d &center, double radius) : center(center), radius(radius) {}


OMPLSphereShellWrapper::OMPLSphereShellWrapper(std::shared_ptr<CollisionFreeShell> shell, ompl::base::SpaceInformationPtr si) :
        shell(std::move(shell)), si(std::move(si)) {}

void OMPLSphereShellWrapper::state_on_shell(const Apple &apple, ompl::base::State* st) const {
    auto state_space = std::static_pointer_cast<ompl_interface::ModelBasedStateSpace>(si->getStateSpace());
    state_space->copyToOMPLState(st, shell->state_on_shell(state_space->getRobotModel(), apple));
}

ompl::geometric::PathGeometric OMPLSphereShellWrapper::path_on_shell(const Apple &a, const Apple &b) {
    return omplPathFromMoveitTrajectory(shell->path_on_shell(si->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->getRobotModel(),a,b),si);
}

std::shared_ptr<CollisionFreeShell> OMPLSphereShellWrapper::getShell() const {
    return shell;
}

void OMPLSphereShellWrapper::state_on_shell(const ompl::base::Goal *apple, ompl::base::State *st) const {
    return state_on_shell(
            Apple { apple->as<DroneEndEffectorNearTarget>()->getTarget(), {0.0,0.0,0.0} },
            st
    );
}

ompl::geometric::PathGeometric
OMPLSphereShellWrapper::path_on_shell(const ompl::base::Goal *a, const ompl::base::Goal *b) {
    return path_on_shell(
            Apple { a->as<DroneEndEffectorNearTarget>()->getTarget(), {0.0,0.0,0.0} },
            Apple { b->as<DroneEndEffectorNearTarget>()->getTarget(), {0.0,0.0,0.0} }
    );
}

