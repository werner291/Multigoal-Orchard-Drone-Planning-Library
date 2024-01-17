// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 1/2/24.
//

#ifndef MGODPL_DUALPRIORITYQUEUE_H
#define MGODPL_DUALPRIORITYQUEUE_H

#include <set>
#include <queue>

/**
 * A Priority Queue that supports two types of elements: refindable and non-refindable.
 *
 * Re-findable events are those that can be found back relatively quickly,
 * whereas non-refindable events are those that cannot be found back quickly,
 * but are much faster to insert and pop-off.
 *
 * @tparam T	The type of the elements in the queue. (Must be comparable with operator<).
 */
template <typename T>
class DualPriorityQueue {

public:
	/// The set of events that can be found back relatively quickly.
	std::set<T> _refindable;
	/// The set of events that should only be used for inserting once and popping once.
	std::priority_queue<T, std::vector<T>, std::greater<T>> _non_refindable;

	/// Insert an element into the queue; it can be found back.
	bool insert_refindable(const T& element) {
		return _refindable.insert(element).second;
	}

	/// Insert an element into the queue; it cannot be found back efficiently.
	void insert_non_refindable(const T& element) {
		return _non_refindable.push(element);
	}

	/// Check whether an event inserted with insert_refindable is in the queue.
	bool contains_refindable(const T& element) const {
		return _refindable.find(element) != _refindable.end();
	}

	/// Do a slow check whether an event inserted with insert_non_refindable is in the queue.
	/// Warning: this is meant for debugging purposes only.
	bool slow_contains_nonrefindable(const T& element) const {
		throw std::runtime_error("Not implemented");
//		for (const T& e : _non_refindable) {
//			if (e == element) {
//				return true;
//			}
//		}
//		return false;
	}

	/// Erase an event inserted with insert_refindable.
	bool erase_refindable(const T& element) {
		return _refindable.erase(element) > 0;
	}

	/// Check if both queues are empty.
	[[nodiscard]] bool empty() const {
		return _refindable.empty() && _non_refindable.empty();
	}

	/**
	 * Pop the first event.
	 * @return 		The first event.
	 */
	T pop_first() {

		// We should always have at least one element.
		assert(!empty());

		// If refindable is empty, we can just pop from non-refindable.
		if (_refindable.empty()) {
			T element = _non_refindable.top();
			_non_refindable.pop();
			return element;
		}

		// If non-refindable is empty, we can just pop from refindable.
		if (_non_refindable.empty()) {
			auto it = _refindable.begin();
			T element = *it;
			_refindable.erase(it);
			return element;
		}

		// Otherwise, grab the start of both queues and pop the smallest one.
		auto it_refindable = _refindable.begin();
		T non_refindable = _non_refindable.top();

		// Pop from the smallest one.
		if (*it_refindable < non_refindable) {
			T element = *it_refindable;
			_refindable.erase(it_refindable);
			return element;
		} else {
			_non_refindable.pop();
			return non_refindable;
		}

	}

	const T& peek_first() const {
		// We should always have at least one element.
		assert(!empty());

		// If refindable is empty, we can just peek from non-refindable.
		if (_refindable.empty()) {
			return _non_refindable.top();
		}

		// If non-refindable is empty, we can just peek from refindable.
		if (_non_refindable.empty()) {
			return *_refindable.begin();
		}

		// Otherwise, grab the start of both queues and peek the smallest one.
		auto it_refindable = _refindable.begin();
		const T& non_refindable = _non_refindable.top();

		// Peek the smallest one.
		if (*it_refindable < non_refindable) {
			return *it_refindable;
		} else {
			return _non_refindable.top();
		}
	}

	const std::set<T>& refindable() const {
		return _refindable;
	}

	const std::priority_queue<T>& non_refindable() const {
		return _non_refindable;
	}

};


#endif //MGODPL_DUALPRIORITYQUEUE_H
