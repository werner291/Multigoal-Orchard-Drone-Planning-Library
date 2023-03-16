//
// Created by werner on 16-3-23.
//

#ifndef NEW_PLANNERS_SIZE_T_IOTA_H
#define NEW_PLANNERS_SIZE_T_IOTA_H


class size_t_iota_view : public ranges::view_facade<size_t_iota_view> {
	friend ranges::range_access;
	size_t value_, end_;

	struct cursor {
		size_t value;

		size_t read() const noexcept {
			return value;
		}

		bool equal(const cursor& that) const noexcept {
			return value == that.value;
		}

		void next() noexcept {
			++value;
		}
	};

	cursor begin_cursor() const noexcept {
		return {value_};
	}

	cursor end_cursor() const noexcept {
		return {end_};
	}

public:
	size_t_iota_view() = default;
	size_t_iota_view(size_t from, size_t to) noexcept : value_(from), end_(to) {}
};

size_t_iota_view size_t_iota(size_t from, size_t to) noexcept {
	return {from, to};
}

#endif //NEW_PLANNERS_SIZE_T_IOTA_H
