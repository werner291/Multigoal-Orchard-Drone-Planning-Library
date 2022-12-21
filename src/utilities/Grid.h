// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 21-12-22.
//

#ifndef NEW_PLANNERS_GRID_H
#define NEW_PLANNERS_GRID_H

#include <utility>

/**
 * @brief Represents a 2D grid with a fixed width and height, and provides an iterator
 */
class Grid {
public:
	// Constructor that initializes the grid with a given width and height
	Grid(int width, int height) : width_(width), height_(height) {
	}

	// Iterator class that allows you to iterate over all pairs of x/y values within the grid
	class iterator {
	public:
		// Constructor that initializes the iterator with a starting position
		iterator(int x, int y, const Grid &grid) : x_(x), y_(y), grid_(grid) {
		}

		// Overload the ++ operator to advance the iterator to the next position
		iterator &operator++() {
			x_++;
			if (x_ == grid_.width_) {
				x_ = 0;
				y_++;
			}
			return *this;
		}

		// Overload the != operator to compare two iterators
		bool operator!=(const iterator &other) const {
			return x_ != other.x_ || y_ != other.y_;
		}

		// Overload the * operator to return a reference to the current position
		std::pair<int, int> operator*() const {
			return std::make_pair(x_, y_);
		}

	private:
		int x_, y_;
		const Grid &grid_;
	};

	// Returns an iterator to the first position in the grid
	iterator begin() const {
		return iterator(0, 0, *this);
	}

	// Returns an iterator to the position past the last element in the grid
	iterator end() const {
		return iterator(0, height_, *this);
	}

private:
	int width_, height_;
};

#endif //NEW_PLANNERS_GRID_H
