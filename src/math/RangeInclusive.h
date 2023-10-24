// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/16/23.
//

#ifndef MGODPL_RANGEINCLUSIVE_H
#define MGODPL_RANGEINCLUSIVE_H

#include <optional>

namespace mgodpl::math {

	template<typename Scalar>
	struct RangeInclusive {
		Scalar min, max;

		RangeInclusive(Scalar min, Scalar max) : min(min), max(max) { // NOLINT(*-easily-swappable-parameters)
		}

		[[nodiscard]] RangeInclusive clamp(const RangeInclusive &other) const {
			return RangeInclusive(std::max(min, other.min), std::min(max, other.max));
		}

		[[nodiscard]] bool contains(Scalar value) const {
			return min <= value && value <= max;
		}

		void expand(const Scalar &d) {
			min = std::min(min, d);
			max = std::max(max, d);
		}

		std::optional<RangeInclusive> intersect(RangeInclusive inclusive) const {
			// If overlapping, return the intersection.
			if (inclusive.min <= max && min <= inclusive.max) {
				return std::make_optional(RangeInclusive(std::max(min, inclusive.min), std::min(max, inclusive.max)));
			} else {
				return std::nullopt;
			}
		}
	};

	typedef RangeInclusive<double> RangeInclusiveD;
	typedef RangeInclusive<int> RangeInclusiveI;

}

#endif //MGODPL_RANGEINCLUSIVE_H
