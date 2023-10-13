
#include <gtest/gtest.h>
#include "../src/utilities/experiment_utils.h"
#include "../src/TreeMeshes.h"
#include "../src/vtk/SimulatedSensor.h"
#include "../src/utilities/vtk.h"
#include "../src/exploration/VtkToPointCloud.h"

#include <range/v3/all.hpp>

/**
 * Sensor that repeatedly flips the orientation of a SimulatedSensor and makes sure that
 * the returned point cloud correctly reflects the orientation, including whether or not
 * it can see the tree.
 */
TEST(VisualiserTest, SensorSyncTest) {

	SimplifiedOrchard orchard = {
			{
					{ { 0.0, 0.0 }, loadTreeMeshes("appletree") },
			}
	};

	/**
	 * PHOTOSENITIVITY WARNING: THIS TEST CAUSES SEVERE FLICKERING IN THE OUTPUT WINDOW WHEN `showWindow` IS TRUE.
	 */
	SimulatedSensor sensor(false);

	sensor.addActorCollection(buildOrchardActors(orchard, true));

	Eigen::Vector3d pos(0.0,5.0,1.5);

	for (size_t i = 0; i < 100; ++i) {
		Eigen::Isometry3d sensor_pose;
		sensor_pose.setIdentity();
		sensor_pose.translation() = pos;

		if (i % 2 == 0) {
			sensor_pose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
		}

		sensor.renderSnapshot(sensor_pose);

		SegmentedPointCloud segmentedPointCloud = segmentPointCloudData(sensor.getPointCloud());

		bool has_leaves = false;
		for (const auto &pt: segmentedPointCloud.points) {
			if (pt.type == SegmentedPointCloud::PT_SOFT_OBSTACLE) {
				has_leaves = true;
				break;
			}
		}

		ASSERT_EQ(has_leaves, i % 2 == 0);

	}

}