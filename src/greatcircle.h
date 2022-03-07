//
// Created by werner on 02-03-22.
//

#ifndef NEW_PLANNERS_GREATCIRCLE_H
#define NEW_PLANNERS_GREATCIRCLE_H

class GreatCircleMetric {
    const Eigen::Vector3d sphere_center;
//    double sphere_radius;

public:
    explicit GreatCircleMetric(Eigen::Vector3d sphereCenter);

    [[nodiscard]] double measure(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
};

#endif //NEW_PLANNERS_GREATCIRCLE_H
