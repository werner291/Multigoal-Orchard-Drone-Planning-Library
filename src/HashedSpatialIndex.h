

#ifndef NEW_PLANNERS_HASHEDSPATIALINDEX_H
#define NEW_PLANNERS_HASHEDSPATIALINDEX_H

#include <vector>
#include <Eigen/Core>
#include <optional>

/**
 * A spatial index that stores points in a hash table in an associative V3-Value fashion.
 *
 * For b buckets and n entries, the average time complexity of a query is O(n/b).
 *
 * @tparam V 		The type of the values stored in the index.
 */
template<typename V>
class HashedSpatialIndex {

	std::vector<std::vector<std::pair<Eigen::Vector3d,V>>> spatial_hash_index;
	const double resolution;

	[[nodiscard]] size_t hashCellIndex(const Eigen::Vector3i &rounded) const {
		return (rounded.x() + rounded.y() * 31 + rounded.z() * 31 * 31) % spatial_hash_index.size();
	}

	[[nodiscard]] Eigen::Vector3i cellIndex(const Eigen::Vector3d &point) const {
		Eigen::Vector3i rounded = (point / resolution).cast<int>();
		return rounded;
	}

public:
	/**
	 * Create a new HashedSpatialIndex.
	 *
	 * @param resolution 		The resolution of the index. Cells are of size resolution^3.
	 * @param table_size 		The number of buckets in the hash table.
	 */
	HashedSpatialIndex(double resolution, size_t table_size) : spatial_hash_index(table_size), resolution(resolution) {}

	/**
	 * Insert a value into the index.
	 * @param point 	The associated point.
	 * @param value 	The value to insert.
	 */
	void insert(const Eigen::Vector3d &point, V value) {
		Eigen::Vector3i rounded = cellIndex(point);
		size_t hash = hashCellIndex(rounded);
		spatial_hash_index[hash].push_back({
			point, std::move(value)
		});
	}

	/**
	 * Query the index for any key-value pairs that are within a given radius of a point. Returns the first found value.
	 * @param point 			The point to query around.
	 * @param max_distance 		The maximum distance from the point to search. ( must be < resolution )
	 * @return 					An optional containing the first found value, or std::nullopt if no value was found.
	 */
	[[nodiscard]] std::optional<std::pair<Eigen::Vector3d,V>> any_within(const Eigen::Vector3d &point, double max_distance) const {
		assert(max_distance <= resolution);

		Eigen::Vector3i rounded = cellIndex(point);

		for (int x = -1; x <= 1; x++) {
			for (int y = -1; y <= 1; y++) {
				for (int z = -1; z <= 1; z++) {
					Eigen::Vector3i cell = rounded + Eigen::Vector3i(x, y, z);
					size_t hash = hashCellIndex(cell);
					for (const auto &entry: spatial_hash_index[hash]) {
						if ((entry.first - point).squaredNorm() <= max_distance*max_distance) {
							return entry;
						}
					}
				}
			}
		}

		return {};
	}


};

#endif //NEW_PLANNERS_HASHEDSPATIALINDEX_H
