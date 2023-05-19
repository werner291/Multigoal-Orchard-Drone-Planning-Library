// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 19-5-23.
//

#ifndef NEW_PLANNERS_OPTIMIZATION_META_H
#define NEW_PLANNERS_OPTIMIZATION_META_H

#include <boost/circular_buffer.hpp>

/**
 * @brief Applies a metaheuristic optimization approach to improve a given solution.
 *
 * The optimization function is applied iteratively to improve the solution, with an adaptively scaled parameter.
 * The termination condition is based solely on the relative improvement over the last X iterations.
 *
 * @tparam SolutionType The type of the solution to be optimized.
 * @tparam OptimizationFunctionType The type of the function that performs a single optimization step.
 * @tparam QualityScoreFunctionType The type of the function that calculates the quality score of a solution.
 *
 * @param solution The initial solution, which will be updated in-place by the optimization function.
 * @param optimize The function that performs a single optimization step.
 * @param qualityScore The function that calculates the quality score of a solution.
 * @param startT The initial value for the adaptive parameter used in the optimization function.
 * @param relativeImprovementThreshold The threshold for the relative improvement, below which the optimization stops.
 * @param numIterations The number of iterations over which the relative improvement is considered for the stopping criterion.
 */
template<typename SolutionType, typename OptimizationFunctionType, typename QualityScoreFunctionType>
void metaheuristic(SolutionType &solution,
				   OptimizationFunctionType optimize,
				   QualityScoreFunctionType qualityScore,
				   double startT = 1.0,
				   double relativeImprovementThreshold = 1e-6,
				   int numIterations = 5) {
	double t = startT;  // Start with t = startT
	SolutionType lastSolution = solution;  // Keep track of the last solution
	double lastQuality = qualityScore(lastSolution);  // Calculate the initial quality score
	boost::circular_buffer<double> scoresBuffer(numIterations);  // Circular buffer to hold the quality scores of the last numIterations iterations

	do {
		// Optimize the solution
		bool improved = optimize(solution, t);
		double currentQuality = qualityScore(solution);

		// Calculate the absolute and relative improvement
		double absoluteImprovement = lastQuality - currentQuality;
		double relativeImprovement = absoluteImprovement / lastQuality;

		// Push the current quality score into the buffer (old values get automatically removed)
		scoresBuffer.push_back(currentQuality);

		// If the solution didn't improve, or the absolute improvement is less than the threshold, halve t
		if (!improved || absoluteImprovement < relativeImprovementThreshold) {
			t *= 0.5;
		}

		// Update the last solution and the last quality score
		lastSolution = solution;
		lastQuality = currentQuality;

		// If the buffer is full and the relative improvement over the last numIterations iterations is below the threshold, break the loop
		if (scoresBuffer.) {
			break;
		}
	} while (true);  // The loop stops when the relative improvement over the last numIterations iterations falls below the threshold
}

#endif //NEW_PLANNERS_OPTIMIZATION_META_H
