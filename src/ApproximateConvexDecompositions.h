

#ifndef NEW_PLANNERS_APPROXIMATECONVEXDECOMPOSITIONS_H
#define NEW_PLANNERS_APPROXIMATECONVEXDECOMPOSITIONS_H

/**
 *
 * Approximate convex decomposition streaming algorithm.
 *
 * This algorithm is inspired by the streaming convex hull algorithm.
 *
 * Let D be a set of direction vectors, as a parameter of the algorithm.
 *
 * Create an empty P set of direction-point maps; every set P' in P is a map from D to a point,
 * representing a single approximate convex hull.
 *
 * For each point p in space, as it is received:
 *
 *     If p lies in the interior of any P' in P, then discard p.
 *
 *     Else, find the set P' in P such that the addition of p to P' results in the smallest Hausdorff distance
 *     between the surface of the approximate convex hull before and after adding.
 *
 *     If no such P' exists, or if the Hausdorff distance is greater than a threshold, then create a new P' in P
 *     containing only p.
 *
 *
 */
class ApproximateConvexDecompositions {

};


#endif //NEW_PLANNERS_APPROXIMATECONVEXDECOMPOSITIONS_H
