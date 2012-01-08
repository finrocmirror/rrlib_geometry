//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    tBoundingBox.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2009-05-25
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <limits>

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
// tBoundingBox constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tBoundingBox<Tdimension, TElement>::tBoundingBox()
{
  this->Reset();
}

template <size_t Tdimension, typename TElement>
template <typename TIterator>
tBoundingBox<Tdimension, TElement>::tBoundingBox(TIterator begin, TIterator end)
{
  this->Reset();
  this->Add(begin, end);
}

//----------------------------------------------------------------------
// tBoundingBox Reset
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tBoundingBox<Tdimension, TElement>::Reset()
{
  this->min = tPoint::Identity() * std::numeric_limits<TElement>::max();
  this->max = tPoint::Identity() * -std::numeric_limits<TElement>::max();
}

//----------------------------------------------------------------------
// tBoundingBox Add
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tBoundingBox<Tdimension, TElement>::Add(const tPoint &point)
{
  for (size_t i = 0; i < Tdimension; ++i)
  {
    this->min[i] = std::min(this->min[i], point[i]);
    this->max[i] = std::max(this->max[i], point[i]);
  }
}

template <size_t Tdimension, typename TElement>
template <typename TIterator>
void tBoundingBox<Tdimension, TElement>::Add(TIterator begin, TIterator end)
{
  for (auto it = begin; it != end; ++it)
  {
    this->Add(*it);
  }
}

template <size_t Tdimension, typename TElement>
void tBoundingBox<Tdimension, TElement>::Add(const tBoundingBox &other)
{
  this->Add(other.Min());
  this->Add(other.Max());
}

//----------------------------------------------------------------------
// tBoundingBox Intersects
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const bool tBoundingBox<Tdimension, TElement>::Intersects(const tBoundingBox &other) const
{
  for (size_t i = 0; i < Tdimension; ++i)
  {
    if ((this->min[i] > other.max[i]) || (other.min[i] > this->max[i]))
    {
      return false;
    }
  }

  return true;
}

//----------------------------------------------------------------------
// tBoundingBox GetIntersection
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const tBoundingBox<Tdimension, TElement> tBoundingBox<Tdimension, TElement>::GetIntersection(const tBoundingBox &other) const
{
  tBoundingBox<Tdimension, TElement> result;
  for (size_t i = 0; i < Tdimension; ++i)
  {
    result.min[i] = std::min(this->min[i], other.min[i]);
    result.max[i] = std::max(this->max[i], other.max[i]);
  }

  return result;
}

//----------------------------------------------------------------------
// tBoundingBox Contains
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const bool tBoundingBox<Tdimension, TElement>::Contains(const tPoint &point) const
{
  for (size_t i = 0; i < Tdimension; ++i)
  {
    if ((this->min[i] > point[i]) || (point[i] > this->max[i]))
    {
      return false;
    }
  }
  return true;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
