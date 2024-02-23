// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 23-2-24.
//

#ifndef MGODPL_PROMPTING_H
#define MGODPL_PROMPTING_H

#pragma once

/// A macro to prompt the user for a value of a certain type.
/// If the user input is invalid, the user is prompted again.
#define PROMPT_USER(type, name, description) \
	type name; \
	std::cout << description << std::endl; \
	while (!(std::cin >> name)) { \
		std::cin.clear(); \
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); \
		std::cout << "Invalid input. Please enter a valid " << #type << "." << std::endl; \
	}

/// A macro to prompt the user for a value of a certain type, with a validation expression.
#define PROMPT_USER_VALIDATED(type, name, description, validation) \
	type name; \
	std::cout << description << std::endl; \
	while (!(std::cin >> name) || !(validation)) { \
		std::cin.clear(); \
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); \
		std::cout << "Invalid input. Please enter a valid " << #type << " compliant with " << #validation << "." << std::endl; \
	}

#endif //MGODPL_PROMPTING_H
