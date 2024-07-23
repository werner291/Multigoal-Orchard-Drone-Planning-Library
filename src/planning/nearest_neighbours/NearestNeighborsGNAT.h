// Copyright (c) 2024 University College Roosevelt
//
// All rights reserved.

//
// Created by werner on 7/23/24.
//

#ifndef GNAT_H
#define GNAT_H
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mark Moll, Bryant Gipson */
/* Modified by Werner Kroneman to remove OMPL dependencies. */

#include <algorithm>
#include <iostream>
#include <queue>
#include <random>
#include <unordered_set>
#include <utility>

#include "NearestNeighbors.h"

namespace ompl {
    /**
     * A spatial index of a set of points in a metric space allowing nearest neighbor queries.
     * @tparam T    The type of elements stored in the data structure
     */
    template<typename T>
    class NearestNeighborsGNAT : public NearestNeighbors<T> {
    protected:
        // internally, we use a priority queue for nearest neighbors, paired
        // with their distance to the query point
        using NearQueue = std::priority_queue<std::pair<double, const T *> >;

        // another internal data structure is a priority queue of nodes to
        // check next for possible nearest neighbors
        class Node;
        using NodeDist = std::pair<Node *, double>;

        struct NodeDistCompare {
            bool operator()(const NodeDist &n0, const NodeDist &n1) const {
                return (n0.second - n0.first->maxRadius_) > (n1.second - n1.first->maxRadius_);
            }
        };

        using NodeQueue = std::priority_queue<NodeDist, std::vector<NodeDist>, NodeDistCompare>;

    public:
        /// A function that selects the pivots for the GNAT when splitting a node.
        using SelectPivotFn = std::function<std::vector<size_t>(const std::vector<T> &, size_t)>;

        explicit NearestNeighborsGNAT(
            SelectPivotFn fn,
            unsigned int degree = 8,
            unsigned int minDegree = 4,
            unsigned int maxDegree = 12,
            unsigned int maxNumPtsPerLeaf = 50,
            unsigned int removedCacheSize = 500,
            bool rebalancing = false
        )
            : NearestNeighbors<T>()
              , pivotSelector_(std::move(fn))
              , degree_(degree)
              , minDegree_(std::min(degree, minDegree))
              , maxDegree_(std::max(maxDegree, degree))
              , maxNumPtsPerLeaf_(maxNumPtsPerLeaf)
              , rebuildSize_(rebalancing ? maxNumPtsPerLeaf * degree : std::numeric_limits<std::size_t>::max())
              , removedCacheSize_(removedCacheSize) {
        }

        ~NearestNeighborsGNAT() override {
            delete tree_;
        }

        void clear() override {
            if (tree_) {
                delete tree_;
                tree_ = nullptr;
            }
            size_ = 0;
            removed_.clear();
            if (rebuildSize_ != std::numeric_limits<std::size_t>::max())
                rebuildSize_ = maxNumPtsPerLeaf_ * degree_;
        }

        bool reportsSortedResults() const override {
            return true;
        }

        void add(const T &data) override {
            if (tree_) {
                if (isRemoved(data))
                    rebuildDataStructure();
                tree_->add(*this, data);
            } else {
                tree_ = new Node(degree_, maxNumPtsPerLeaf_, data);
                size_ = 1;
            }
        }

        void add(const std::vector<T> &data) override {
            if (tree_)
                NearestNeighbors<T>::add(data);
            else if (!data.empty()) {
                tree_ = new Node(degree_, maxNumPtsPerLeaf_, data[0]);
#ifdef GNAT_SAMPLER
                 tree_->subtreeSize_ = data.size();
#endif
                tree_->data_.insert(tree_->data_.end(), data.begin() + 1, data.end());
                size_ += data.size();
                if (tree_->needToSplit(*this))
                    tree_->split(*this);
            }
        }

        void rebuildDataStructure() {
            std::vector<T> lst;
            list(lst);
            clear();
            add(lst);
        }

        bool remove(const T &data) override {
            if (size_ == 0u)
                return false;
            NearQueue nbhQueue;
            // find data in tree
            bool isPivot = nearestKInternal(data, 1, nbhQueue);
            const T *d = nbhQueue.top().second;
            if (*d != data)
                return false;
            removed_.insert(d);
            size_--;
            // if we removed a pivot or if the capacity of removed elements
            // has been reached, we rebuild the entire GNAT
            if (isPivot || removed_.size() >= removedCacheSize_)
                rebuildDataStructure();
            return true;
        }

        T nearest(const T &data) const override {
            if (size_) {
                NearQueue nbhQueue;
                nearestKInternal(data, 1, nbhQueue);
                if (!nbhQueue.empty())
                    return *nbhQueue.top().second;
            }
            throw std::runtime_error("No elements found in nearest neighbors data structure");
        }

        void nearestK(const T &data, std::size_t k, std::vector<T> &nbh) const override {
            nbh.clear();
            if (k == 0)
                return;
            if (size_) {
                NearQueue nbhQueue;
                nearestKInternal(data, k, nbhQueue);
                postprocessNearest(nbhQueue, nbh);
            }
        }

        void nearestR(const T &data, double radius, std::vector<T> &nbh) const override {
            nbh.clear();
            if (size_) {
                NearQueue nbhQueue;
                nearestRInternal(data, radius, nbhQueue);
                postprocessNearest(nbhQueue, nbh);
            }
        }

        std::size_t size() const override {
            return size_;
        }

        void list(std::vector<T> &data) const override {
            data.clear();
            data.reserve(size());
            if (tree_)
                tree_->list(*this, data);
        }

        friend std::ostream &operator<<(std::ostream &out, const NearestNeighborsGNAT<T> &gnat) {
            if (gnat.tree_) {
                out << *gnat.tree_;
                if (!gnat.removed_.empty()) {
                    out << "Elements marked for removal:\n";
                    for (const auto &elt: gnat.removed_)
                        out << *elt << '\t';
                    out << std::endl;
                }
            }
            return out;
        }

        // for debugging purposes
        void integrityCheck() {
            std::vector<T> lst;
            std::unordered_set<const T *> tmp;
            // get all elements, including those marked for removal
            removed_.swap(tmp);
            list(lst);
            // check if every element marked for removal is also in the tree
            for (const auto &elt: tmp) {
                unsigned int i;
                for (i = 0; i < lst.size(); ++i)
                    if (lst[i] == *elt)
                        break;
                if (i == lst.size()) {
                    // an element marked for removal is not actually in the tree
                    std::cout << "***** FAIL!! ******\n" << *this << '\n';
                    for (const auto &l: lst)
                        std::cout << l << '\t';
                    std::cout << std::endl;
                }
                assert(i != lst.size());
            }
            // restore
            removed_.swap(tmp);
            // get elements in the tree with elements marked for removal purged from the list
            list(lst);
            if (lst.size() != size_)
                std::cout << "#########################################\n" << *this << std::endl;
            assert(lst.size() == size_);
        }

    protected:
        using GNAT = NearestNeighborsGNAT<T>;

        bool isRemoved(const T &data) const {
            return !removed_.empty() && removed_.find(&data) != removed_.end();
        }

        bool nearestKInternal(const T &data, std::size_t k, NearQueue &nbhQueue) const {
            bool isPivot;
            double dist;
            NodeDist nodeDist;
            NodeQueue nodeQueue;

            dist = NearestNeighbors<T>::distFun_(data, tree_->pivot_);
            isPivot = tree_->insertNeighborK(nbhQueue, k, tree_->pivot_, data, dist);
            tree_->nearestK(*this, data, k, nbhQueue, nodeQueue, isPivot);
            while (!nodeQueue.empty()) {
                dist = nbhQueue.top().first; // note the difference with nearestRInternal
                nodeDist = nodeQueue.top();
                nodeQueue.pop();
                if (nbhQueue.size() == k && (nodeDist.second > nodeDist.first->maxRadius_ + dist ||
                                             nodeDist.second < nodeDist.first->minRadius_ - dist))
                    continue;
                nodeDist.first->nearestK(*this, data, k, nbhQueue, nodeQueue, isPivot);
            }
            return isPivot;
        }

        void nearestRInternal(const T &data, double radius, NearQueue &nbhQueue) const {
            double dist = radius; // note the difference with nearestKInternal
            NodeQueue nodeQueue;
            NodeDist nodeDist;

            tree_->insertNeighborR(nbhQueue,
                                   radius,
                                   tree_->pivot_,
                                   NearestNeighbors<T>::distFun_(data, tree_->pivot_));
            tree_->nearestR(*this, data, radius, nbhQueue, nodeQueue);
            while (!nodeQueue.empty()) {
                nodeDist = nodeQueue.top();
                nodeQueue.pop();
                if (nodeDist.second > nodeDist.first->maxRadius_ + dist ||
                    nodeDist.second < nodeDist.first->minRadius_ - dist)
                    continue;
                nodeDist.first->nearestR(*this, data, radius, nbhQueue, nodeQueue);
            }
        }

        void postprocessNearest(NearQueue &nbhQueue, std::vector<T> &nbh) const {
            nbh.resize(nbhQueue.size());
            for (auto it = nbh.rbegin(); it != nbh.rend(); it++, nbhQueue.pop())
                *it = *nbhQueue.top().second;
        }

        class Node {
        public:
            Node(int degree, int capacity, T pivot)
                : degree_(degree)
                  , pivot_(std::move(pivot))
                  , minRadius_(std::numeric_limits<double>::infinity())
                  , maxRadius_(-minRadius_)
                  , minRange_(degree, minRadius_)
                  , maxRange_(degree, maxRadius_) {
                // The "+1" is needed because we add an element before we check whether to split
                data_.reserve(capacity + 1);
            }

            ~Node() {
                for (auto &child: children_)
                    delete child;
            }

            void updateRadius(double dist) {
                if (minRadius_ > dist)
                    minRadius_ = dist;
                if (maxRadius_ < dist)
                    maxRadius_ = dist;
            }

            void updateRange(unsigned int i, double dist) {
                if (minRange_[i] > dist)
                    minRange_[i] = dist;
                if (maxRange_[i] < dist)
                    maxRange_[i] = dist;
            }

            void add(GNAT &gnat, const T &data) {
                if (children_.empty()) {
                    data_.push_back(data);
                    gnat.size_++;
                    if (needToSplit(gnat)) {
                        if (!gnat.removed_.empty())
                            gnat.rebuildDataStructure();
                        else if (gnat.size_ >= gnat.rebuildSize_) {
                            gnat.rebuildSize_ <<= 1;
                            gnat.rebuildDataStructure();
                        } else
                            split(gnat);
                    }
                } else {
                    std::vector<double> dist(children_.size());
                    double minDist = dist[0] = gnat.distFun_(data, children_[0]->pivot_);
                    int minInd = 0;

                    for (unsigned int i = 1; i < children_.size(); ++i)
                        if ((dist[i] = gnat.distFun_(data, children_[i]->pivot_)) < minDist) {
                            minDist = dist[i];
                            minInd = i;
                        }
                    for (unsigned int i = 0; i < children_.size(); ++i)
                        children_[i]->updateRange(minInd, dist[i]);
                    children_[minInd]->updateRadius(minDist);
                    children_[minInd]->add(gnat, data);
                }
            }

            bool needToSplit(const GNAT &gnat) const {
                unsigned int sz = data_.size();
                return sz > gnat.maxNumPtsPerLeaf_ && sz > degree_;
            }

            /**
             * Split this node.
             * @param gnat The GNAT object that this is a node of, to access the pivot selector and such.
             */
            void split(GNAT &gnat) {
                // Check that this isn't a leaf node.
                assert(children_.empty());

                // This procedure will split the current node into degree_ children.

                // First, we need to find the degree_ pivots. That is, we want to pick degree_ elements from the data
                // that are somewhat far apart from each other.
                const auto &pivots = gnat.pivotSelector_(data_, degree_);
                degree_ = pivots.size(); // in case fewer than degree_ pivots were found

                // Now, we create the children: one for each pivot.
                children_.reserve(degree_);
                for (const auto &pivot: pivots)
                    children_.push_back(new Node(degree_, gnat.maxNumPtsPerLeaf_, data_[pivot]));

                for (unsigned int j = 0; j < data_.size(); ++j) {
                    // Find the index of the closest pivot to the data point.
                    unsigned int closest_pivot = std::min_element(pivots.begin(),
                                                                  pivots.end(),
                                                                  [&](unsigned int i, unsigned int j) {
                                                                      return gnat.distFun_(data_[i], data_[j]) < gnat.
                                                                             distFun_(
                                                                                 data_[j],
                                                                                 data_[i]);
                                                                  }) - pivots.begin();

                    // Copy the data point to the child that is closest to it; do not include the pivot itself.
                    Node *child = children_[closest_pivot];
                    if (j != pivots[closest_pivot]) {
                        child->data_.push_back(data_[j]);
                        // Update the radius of the child with the distance between the new data point and the pivot.
                        // (That is: the min/max radius)
                        child->updateRadius(gnat.distFun_(data_[j], child->pivot_));
                    }
                    for (unsigned int i = 0; i < degree_; ++i)
                        children_[i]->updateRange(closest_pivot, gnat.distFun_(data_[j], children_[i]->pivot_));
                }

                for (auto &child: children_) {
                    // make sure degree lies between minDegree_ and maxDegree_
                    child->degree_ =
                            std::min(std::max((unsigned int) ((degree_ * child->data_.size()) / data_.size()),
                                              gnat.minDegree_),
                                     gnat.maxDegree_);
                    // singleton
                    if (child->minRadius_ >= std::numeric_limits<double>::infinity())
                        child->minRadius_ = child->maxRadius_ = 0.;
                }
                // this does more than clear(); it also sets capacity to 0 and frees the memory
                std::vector<T> tmp;
                data_.swap(tmp);
                // check if new leaves need to be split
                for (auto &child: children_)
                    if (child->needToSplit(gnat))
                        child->split(gnat);
            }

            bool insertNeighborK(NearQueue &nbh, std::size_t k, const T &data, const T &key, double dist) const {
                if (nbh.size() < k) {
                    nbh.emplace(dist, &data);
                    return true;
                }
                if (dist < nbh.top().first || (dist < std::numeric_limits<double>::epsilon() && data == key)) {
                    nbh.pop();
                    nbh.emplace(dist, &data);
                    return true;
                }
                return false;
            }

            void nearestK(const GNAT &gnat,
                          const T &data,
                          std::size_t k,
                          NearQueue &nbh,
                          NodeQueue &nodeQueue,
                          bool &isPivot) const {
                for (const auto &d: data_)
                    if (!gnat.isRemoved(d)) {
                        if (insertNeighborK(nbh, k, d, data, gnat.distFun_(data, d)))
                            isPivot = false;
                    }
                if (!children_.empty()) {
                    double dist;
                    Node *child;
                    std::size_t sz = children_.size(), offset = gnat.offset_++;
                    std::vector<double> distToPivot(sz);
                    std::vector<int> permutation(sz);
                    for (unsigned int i = 0; i < sz; ++i)
                        permutation[i] = (i + offset) % sz;

                    for (unsigned int i = 0; i < sz; ++i)
                        if (permutation[i] >= 0) {
                            child = children_[permutation[i]];
                            distToPivot[permutation[i]] = gnat.distFun_(data, child->pivot_);
                            if (insertNeighborK(nbh, k, child->pivot_, data, distToPivot[permutation[i]]))
                                isPivot = true;
                            if (nbh.size() == k) {
                                dist = nbh.top().first; // note difference with nearestR
                                for (unsigned int j = 0; j < sz; ++j)
                                    if (permutation[j] >= 0 && i != j &&
                                        (distToPivot[permutation[i]] - dist > child->maxRange_[permutation[j]] ||
                                         distToPivot[permutation[i]] + dist < child->minRange_[permutation[j]]))
                                        permutation[j] = -1;
                            }
                        }

                    dist = nbh.top().first;
                    for (auto p: permutation)
                        if (p >= 0) {
                            child = children_[p];
                            if (nbh.size() < k || (distToPivot[p] - dist <= child->maxRadius_ &&
                                                   distToPivot[p] + dist >= child->minRadius_))
                                nodeQueue.emplace(child, distToPivot[p]);
                        }
                }
            }

            void insertNeighborR(NearQueue &nbh, double r, const T &data, double dist) const {
                if (dist <= r)
                    nbh.emplace(dist, &data);
            }

            void nearestR(const GNAT &gnat, const T &data, double r, NearQueue &nbh, NodeQueue &nodeQueue) const {
                double dist = r; // note difference with nearestK

                for (const auto &d: data_)
                    if (!gnat.isRemoved(d))
                        insertNeighborR(nbh, r, d, gnat.distFun_(data, d));
                if (!children_.empty()) {
                    Node *child;
                    std::size_t sz = children_.size(), offset = gnat.offset_++;
                    std::vector<double> distToPivot(sz);
                    std::vector<int> permutation(sz);
                    // Not a random permutation, but processing the children in slightly different order is
                    // "good enough" to get a performance boost. A call to std::shuffle takes too long.
                    for (unsigned int i = 0; i < sz; ++i)
                        permutation[i] = (i + offset) % sz;

                    for (unsigned int i = 0; i < sz; ++i)
                        if (permutation[i] >= 0) {
                            child = children_[permutation[i]];
                            distToPivot[permutation[i]] = gnat.distFun_(data, child->pivot_);
                            insertNeighborR(nbh, r, child->pivot_, distToPivot[permutation[i]]);
                            for (unsigned int j = 0; j < sz; ++j)
                                if (permutation[j] >= 0 && i != j &&
                                    (distToPivot[permutation[i]] - dist > child->maxRange_[permutation[j]] ||
                                     distToPivot[permutation[i]] + dist < child->minRange_[permutation[j]]))
                                    permutation[j] = -1;
                        }

                    for (auto p: permutation)
                        if (p >= 0) {
                            child = children_[p];
                            if (distToPivot[p] - dist <= child->maxRadius_ &&
                                distToPivot[p] + dist >= child->minRadius_)
                                nodeQueue.emplace(child, distToPivot[p]);
                        }
                }
            }

            void list(const GNAT &gnat, std::vector<T> &data) const {
                if (!gnat.isRemoved(pivot_))
                    data.push_back(pivot_);
                for (const auto &d: data_)
                    if (!gnat.isRemoved(d))
                        data.push_back(d);
                for (const auto &child: children_)
                    child->list(gnat, data);
            }

            unsigned int degree_;
            const T pivot_;
            double minRadius_;
            double maxRadius_;
            std::vector<double> minRange_;
            std::vector<double> maxRange_;
            std::vector<T> data_;
            std::vector<Node *> children_;
        };

        Node *tree_{nullptr};
        unsigned int degree_;
        unsigned int minDegree_;
        unsigned int maxDegree_;
        unsigned int maxNumPtsPerLeaf_;
        std::size_t size_{0};
        std::size_t rebuildSize_;
        std::size_t removedCacheSize_;
        SelectPivotFn pivotSelector_;
        std::unordered_set<const T *> removed_;

        // used to cycle through children of a node in different orders
        mutable std::size_t offset_{0};
    };
}


#endif //GNAT_H
