
#ifndef NEW_PLANNERS_OMPLSHELLSPACE_H
#define NEW_PLANNERS_OMPLSHELLSPACE_H

#include <memory>
#include "MoveItShellSpace.h"

template<typename ShellPoint>
class OmplShellSpace {

	std::shared_ptr<const MoveItShellSpace<ShellPoint>> shell_space;

};


#endif //NEW_PLANNERS_OMPLSHELLSPACE_H
