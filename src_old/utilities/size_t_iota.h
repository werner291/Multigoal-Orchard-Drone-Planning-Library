//
// Created by werner on 16-3-23.
//

#ifndef NEW_PLANNERS_SIZE_T_IOTA_H
#define NEW_PLANNERS_SIZE_T_IOTA_H

#include <range/v3/view_facade.hpp>

class size_t_iota_view : public ranges::view_facade<size_t_iota_view> {
	friend ranges::range_access;
	size_t value_, end_;

	struct cursor {
		size_t value;

		size_t read() const noexcept;

		bool equal(const cursor& that) const noexcept;

		void next() noexcept;
	};

	cursor begin_cursor() const noexcept;

	cursor end_cursor() const noexcept;

public:
	size_t_iota_view() = default;
	size_t_iota_view(size_t from, size_t to) noexcept;
};

size_t_iota_view size_t_iota(size_t from, size_t to) noexcept;

#endif //NEW_PLANNERS_SIZE_T_IOTA_H
