// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

#include "size_t_iota.h"

size_t size_t_iota_view::cursor::read() const noexcept {
	return value;
}

bool size_t_iota_view::cursor::equal(const size_t_iota_view::cursor &that) const noexcept {
	return value == that.value;
}

void size_t_iota_view::cursor::next() noexcept {
	++value;
}

size_t_iota_view::cursor size_t_iota_view::begin_cursor() const noexcept {
	return {value_};
}

size_t_iota_view::cursor size_t_iota_view::end_cursor() const noexcept {
	return {end_};
}

size_t_iota_view::size_t_iota_view(size_t from, size_t to) noexcept: value_(from), end_(to) {}

size_t_iota_view size_t_iota(size_t from, size_t to) noexcept {
	return {from, to};
}
