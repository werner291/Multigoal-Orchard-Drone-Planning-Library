// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 14-4-23.
//

#ifndef NEW_PLANNERS_RX_UTIL_H
#define NEW_PLANNERS_RX_UTIL_H

#include <rxcpp/rx-observable.hpp>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrent>
#include <rxqt.hpp>

/**
 * Given a sequence of items, perform some async function on them through the QtConcurrent framework.
 *
 * If an item comes in while the previous item is still being processed, the previous item is discarded.
 *
 * @tparam T 		The type of the input items
 * @tparam F 		The type of the function to be performed on the items
 * @param func 		The function to be performed on the items, expected to return a future
 * @return 			An observable that emits the results of the function
 */
template<typename T, typename F>
auto map_async_latest(F &&func) {

	return [=](const rxcpp::observable<T> &source) {

		// Create a single shared future watcher
		auto futureWatcher = std::make_shared<QFutureWatcher<typename std::result_of<F(T)>::type>>();

		// Map the source through the function, then assign the result to the future watcher
		source.subscribe([=](const T &input) {
			futureWatcher->setFuture(QtConcurrent::run(func, input));
		});

		// Create an observable that emits the results of the future watcher
		return rxqt::from_signal(futureWatcher.get(),
								 &QFutureWatcher<typename std::result_of<F(T)>::type>::finished).map([=](const auto &ignored) {
			return futureWatcher->result();
		});
	};
}

#endif //NEW_PLANNERS_RX_UTIL_H
