// Copyright (c) 2022 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 10/23/23.
//

#ifndef MGODPL_OCTREE_H
#define MGODPL_OCTREE_H

#include <cstddef>
#include <variant>
#include <array>
#include <memory>

namespace mgodpl::visibility {

	template<typename SD, typename LD>
	class Octree {

	public:
		struct LeafNode;
		struct SplitNode;

		using Node = std::variant<LeafNode, SplitNode>;

		struct LeafNode {
			LD data;
		};

		struct SplitNode {
			SD data;
			std::unique_ptr<std::array<Node,8> > children;
		};

		Node _root;

	};

}

#endif //MGODPL_OCTREE_H
