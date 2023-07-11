#include "../planners/shell_path_planner/Construction.h"
#include "../utilities/experiment_utils.h"
#include "../planner_allocators.h"

int main() {

	using ShellPoint = mgodpl::cgal_utils::CGALMeshPointAndNormal;

	auto planner = dynamic_planner_fre<ShellPoint>(cgalChullShell);

	auto drone = loadRobotModel();

	auto trees = loadAllTreeModels(INT_MAX, 600);


}