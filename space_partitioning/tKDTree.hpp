//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    tKDTree.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2008-11-28
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <algorithm>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace geometry
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tKDTree constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename TIterator, typename Metric>
tKDTree<Tdimension, TElement>::tKDTree(TIterator begin_points, TIterator end_points, Metric metric)
  : root(new tNode(begin_points, end_points, metric))
{}

//----------------------------------------------------------------------
// tKDTree destructor
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tKDTree<Tdimension, TElement>::~tKDTree()
{
  delete this->root;
}

//----------------------------------------------------------------------
// tKDTree Root
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tKDTree<Tdimension, TElement>::tNode &tKDTree<Tdimension, TElement>::Root() const
{
  assert(this->root);
  return *this->root;
}

//----------------------------------------------------------------------
// tKDTree::tNode constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename TIterator, typename Metric>
tKDTree<Tdimension, TElement>::tNode::tNode(TIterator points_begin, TIterator points_end, Metric metric)
  : bounding_box(points_begin, points_end),
    split_axis(SelectSplitAxis(metric)),
    split_value(0.5 * (this->bounding_box.Min()[this->split_axis] + this->bounding_box.Max()[this->split_axis])),
    left_child(0),
    right_child(0),
    number_of_points(std::distance(points_begin, points_end))
{
  if (this->number_of_points > 1)
  {
    std::sort(points_begin, points_end, [this](const tPoint & a, const tPoint & b)
    {
      return a[this->split_axis] < b[this->split_axis];
    });
    TIterator split = points_begin;
    while (split != points_end && (*split)[this->split_axis] <= this->split_value)
    {
      ++split;
    }
    if (split != points_begin && split != points_end)
    {
      this->left_child = new tNode(points_begin, split, metric);
      this->right_child = new tNode(split, points_end, metric);
      assert(this->left_child && this->right_child);
    }
  }

  // additional data
  if (this->IsLeaf())
  {
    // there may be more than one point in a leave
    for (auto it = points_begin; it != points_end; it++)
    {
      for (size_t i = 0; i < Tdimension; i++)
      {
        this->center_of_mass[i] += (*it)[i];
      }
    }
    this->center_of_mass *= 1.0 / this->NumberOfPoints();
  }
  else
  {
    // exploiting the recursive structure
    this->center_of_mass = this->left_child->CenterOfMass() * this->left_child->NumberOfPoints() + this->right_child->CenterOfMass() * this->right_child->NumberOfPoints();
    this->center_of_mass *= 1.0 / this->NumberOfPoints();
  }
}

//----------------------------------------------------------------------
// tKDTree::tNode destructor
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tKDTree<Tdimension, TElement>::tNode::~tNode()
{
  delete this->left_child;
  delete this->right_child;
}

//----------------------------------------------------------------------
// tKDTree::tNode SplitAxis
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
size_t tKDTree<Tdimension, TElement>::tNode::SplitAxis() const
{
  return this->split_axis;
}

//----------------------------------------------------------------------
// tKDTree::tNode SplitValue
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
TElement tKDTree<Tdimension, TElement>::tNode::SplitValue() const
{
  return this->split_value;
}

//----------------------------------------------------------------------
// tKDTree::tNode NumberOfPoints
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
size_t tKDTree<Tdimension, TElement>::tNode::NumberOfPoints() const
{
  return this->number_of_points;
}

//----------------------------------------------------------------------
// tKDTree::tNode IsLeaf
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
bool tKDTree<Tdimension, TElement>::tNode::IsLeaf() const
{
  return this->left_child == 0;
}

//----------------------------------------------------------------------
// tKDTree::tNode BoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tKDTree<Tdimension, TElement>::tNode::tBoundingBox &tKDTree<Tdimension, TElement>::tNode::BoundingBox() const
{
  return this->bounding_box;
}

//----------------------------------------------------------------------
// tKDTree::tNode CenterOfMass
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tKDTree<Tdimension, TElement>::tPoint &tKDTree<Tdimension, TElement>::tNode::CenterOfMass() const
{
  return this->center_of_mass;
}

//----------------------------------------------------------------------
// tKDTree::tNode LeftChild
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tKDTree<Tdimension, TElement>::tNode &tKDTree<Tdimension, TElement>::tNode::LeftChild() const
{
  assert(!this->IsLeaf());
  return *this->left_child;
}

//----------------------------------------------------------------------
// tKDTree::tNode RightChild
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tKDTree<Tdimension, TElement>::tNode &tKDTree<Tdimension, TElement>::tNode::RightChild() const
{
  assert(!this->IsLeaf());
  return *this->right_child;
}

//----------------------------------------------------------------------
// tKDTree::tNode SelectSplitAxis
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename Metric>
size_t tKDTree<Tdimension, TElement>::tNode::SelectSplitAxis(Metric metric) const
{
  tPoint sample_point;
  size_t result = 0;
  TElement max_width = -std::numeric_limits<TElement>::max();
  for (size_t i = 0; i < Tdimension; ++i)
  {
    sample_point[i] = this->bounding_box.Max()[i] - this->bounding_box.Min()[i];
    TElement width = metric(sample_point, tPoint::Zero());
    sample_point[i] = 0;
    if (width > max_width)
    {
      max_width = width;
      result = i;
    }
  }
  return result;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
