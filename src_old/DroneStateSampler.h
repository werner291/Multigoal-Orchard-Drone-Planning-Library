#ifndef NEW_PLANNERS_DRONESTATESAMPLER_H
#define NEW_PLANNERS_DRONESTATESAMPLER_H

/**
 * A custom sampler for the DroneStateSpace that samples states where the robot is upright,
 * with the translation of the base within a box of size defined by translationBound. (see randomizeUprightWithBase)
 *
 * Assumes we're using the drone robot model.
 */
class DroneStateSampler : public ompl::base::StateSampler {

	// Translation bound to use with randomizeUprightWithBase
	double translation_bound;

public:
	/**
	 *
	 * @param space 			The state space to sample from (assumed to be a DroneStateSpace)
	 * @param translationBound 	The bound on the translation of the base (see randomizeUprightWithBase)
	 */
	explicit DroneStateSampler(const ompl::base::StateSpace *space, double translationBound = 10.0);

	/**
	 * Samples a state from the state space uniformly.
	 * @param state The state to be written to.
	 */
	void sampleUniform(ompl::base::State *state) override;

	/**
	 * Samples a state near another state within a given distance.
	 *
	 * @param state 			The state to be written to.
	 * @param near 				The reference state to sample nearby.
	 * @param distance 			The distance from the reference state.
	 */
	void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

	/**
	 * Samples a state with a gaussian distribution around another state. (Not implemented!)
	 *
	 * @param state 			The state to be written to.
	 * @param mean 				The reference state to sample around.
	 * @param stdDev 			The standard deviation of the gaussian distribution.
	 */
	void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;
};


#endif //NEW_PLANNERS_DRONESTATESAMPLER_H
