module;
// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

#include <functional>

export module functional_utils;

export namespace mgodpl {

	using calls_t = unsigned long;

	/**
	 * @brief Wraps a function to count the number of invocations.
	 *
	 * This function takes a function and a reference to an unsigned long.
	 * It returns a new function that increments the unsigned long each time the function is called.
	 *
	 * @tparam A The type of the argument of the function to be wrapped.
	 * @tparam R The return type of the function to be wrapped.
	 * @param fn The function to be wrapped.
	 * @param invocations A reference to an unsigned long that will be incremented each time the function is called.
	 * @return A function that increments the unsigned long and calls the original function each time it is called.
	 */
	template<typename F>
	auto wrap_invocation_counting(F fn, calls_t &invocations) {
		return [fn = std::move(fn), &invocations](auto&&... args) {
			invocations += 1;
			return fn(std::forward<decltype(args)>(args)...);
		};
	}
}
