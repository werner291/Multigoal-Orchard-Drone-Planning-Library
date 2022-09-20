
#ifndef NEW_PLANNERS_CYLINDERSHELL_H
#define NEW_PLANNERS_CYLINDERSHELL_H

#include "WorkspaceShell.h"

struct CylinderShellPoint {
	double angle;
	double height;
};

class CylinderShell : public WorkspaceShell<CylinderShellPoint> {
public:
	Eigen::Vector3d arm_vector(const CylinderShellPoint &p) const override;

	CylinderShellPoint nearest_point_on_shell(const Eigen::Vector3d &p) const override;

	Eigen::Vector3d surface_point(const CylinderShellPoint &p) const override;

};


#endif //NEW_PLANNERS_CYLINDERSHELL_H
