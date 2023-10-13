// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 12-4-23.
//

#ifndef NEW_PLANNERS_CHAIN_QT_FUTURES_H
#define NEW_PLANNERS_CHAIN_QT_FUTURES_H

#include <QtConcurrent/QtConcurrent>

#include <QFutureInterface>
#include <QFutureWatcher>
#include <QFuture>
#include <QObject>
#include <functional>
#include <memory>

#include <type_traits>

/**
 * @brief Chains a QFuture with a continuation function that is executed asynchronously.
 *
 * @tparam Callable The type of the continuation function.
 * @tparam Input The type of the input QFuture result.
 * @param input_future The input QFuture to chain the continuation function to.
 * @param continuation_function The function to execute after the input_future finishes.
 * @return A new QFuture containing the result of the continuation function.
 */
template<typename Callable, typename Input>
QFuture<std::invoke_result_t<Callable, const Input &>>
chainQFuture(QFuture<Input> &input_future, Callable continuation_function) {
	// Deduce the return type of the continuation function when called with an Input object.
	using ReturnType = std::invoke_result_t<Callable, const Input &>;

	// Create a watcher for the input_future.
	auto watcher = std::make_shared<QFutureWatcher<Input>>();
	auto future_iface = std::make_shared<QFutureInterface<ReturnType>>();

	watcher->setFuture(input_future);

	// Connect the watcher's finished signal to a lambda that executes the continuation function.
	QObject::connect(watcher.get(), &QFutureWatcher<Input>::finished, [watcher, continuation_function, future_iface]() {
		// Get the result from the input_future.
		auto result = watcher->result();

		// Run the continuation function asynchronously.
		auto continuation_future = QtConcurrent::run(continuation_function, result);

		// Set up a new watcher for the continuation future.
		auto continuation_watcher = std::make_shared<QFutureWatcher<ReturnType>>();
		continuation_watcher->setFuture(continuation_future);

		// Connect the continuation watcher to report the result and finish the future.
		QObject::connect(continuation_watcher.get(),
						 &QFutureWatcher<ReturnType>::finished,
						 [continuation_watcher, future_iface]() {
							 future_iface->reportResult(continuation_watcher->result());
							 future_iface->reportFinished();
						 });
	});

	// Return the new future containing the result of the continuation function.
	return future_iface->future();
}


#endif //NEW_PLANNERS_CHAIN_QT_FUTURES_H
