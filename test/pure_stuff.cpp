// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 13-6-23.
//

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../src/pure/ShellPathPlanner.h"

template<>
std::string mgodpl::path_reverse(std::string path) {
	return {path.rbegin(), path.rend()};
}

template<>
std::string mgodpl::path_concatenate(std::string paths...) {
	std::string result;
	for (const auto& path : paths) {
		result += path;
	}
	return result;
}


template<>
struct mgodpl::PathTraits<std::string> {

	static std::string concatenate(const std::vector<std::string>& paths) {
		std::string result;
		for (const auto& path : paths) {
			result += path;
		}
		return result;
	}
};

template<>
struct mgodpl::ShellSpaceTraits<std::string> {
	using ShellPoint = char;

	static std::string shell_path(ShellPoint sp1, ShellPoint sp2) {
		return std::string(1, sp1) + std::string(1, sp2);
	}
};


TEST(PureStuff, Test1) {

	using namespace mgodpl;

	ApproachPath<std::string, char> approachPath1 {	"abc", 'A' };

	ApproachToGoal<std::string, char, std::size_t> approachToGoal2 {{"def",	'D'}, 42};

	auto composedPath = composeRetreatShellApproachPath<std::string, std::string>(
			approachPath1,
			approachToGoal2,
			std::string(),
			[](const std::string& path) { return path; }
	);

	EXPECT_EQ(composedPath, "cbaADdef");

}