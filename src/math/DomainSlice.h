// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/18/23.
//

#ifndef MGODPL_DOMAINSLICE_H
#define MGODPL_DOMAINSLICE_H

namespace mgodpl::math {

	/**
	 * A slice of an embedding over some range.
	 *
	 * For instance, if we consider an array to be an embedding of a range of natural numbers to the type of array elements,
	 * then a slice of that embedding would be a subarray of the original array, represented in terms of the index range
	 * into the original array.
	 *
	 * Note: this slice holds a reference to the original embedding, so it is only valid as long as the original embedding
	 * is valid.
	 *
	 * @tparam SliceOf 		The type of the topological embedding.
	 * @tparam Range 		The type of the parameter range.
	 */
	template<typename SliceOf, typename Range> struct DomainSlice {
		SliceOf &slice_of;
		Range range;
	};

}

#endif //MGODPL_DOMAINSLICE_H
