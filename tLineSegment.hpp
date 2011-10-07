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
/*!\file    tLineSegment.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-27
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tLUDecomposition.h"

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
// tLineSegment constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement>::tLineSegment()
{
  this->end[0] = 1;
}

template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement>::tLineSegment(const typename tShape::tPoint &begin, const typename tShape::tPoint &end)
    : tLine(begin, end - begin),
    end(end)
{}

//----------------------------------------------------------------------
// tLineSegment Set
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLineSegment<Tdimension, TElement>::Set(const typename tShape::tPoint &begin, const typename tShape::tPoint &end)
{
  tLine::Set(begin, end - begin);
  this->end = end;
}

//----------------------------------------------------------------------
// tLineSegment GetNearestPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tShape<Tdimension, TElement>::tPoint tLineSegment<Tdimension, TElement>::GetNearestPoint(const typename tShape::tPoint &reference_point) const
{
  math::tVector<Tdimension, TElement> from_begin_to_point(reference_point - this->Begin());
  math::tVector<Tdimension, TElement> from_end_to_point(reference_point - this->End());

  // angle between direction and vector begin->point larger than 90°
  if (from_begin_to_point * this->Direction() < 0)
  {
    return this->Begin();
  }

  // angle between direction and vecot end->point larger than 90°
  if (from_end_to_point * this->Direction() > 0)
  {
    return this->End();
  }

  return tLine::GetNearestPoint(reference_point);
}

//----------------------------------------------------------------------
// tLineSegment Translate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement> &tLineSegment<Tdimension, TElement>::Translate(const math::tVector<Tdimension, TElement> &translation)
{
  tLine::Translate(translation);
  this->end += translation;
  return *this;
}

//----------------------------------------------------------------------
// tLineSegment Rotate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement> &tLineSegment<Tdimension, TElement>::Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation)
{
  tLine::Rotate(rotation);
  this->end = rotation * this->end;
  return *this;
}

//----------------------------------------------------------------------
// tLineSegment Transform
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement> &tLineSegment<Tdimension, TElement>::Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation)
{
  tLine::Transform(transformation);
  this->end = transformation.MultiplyHomogeneously(this->end);
  return *this;
}

//----------------------------------------------------------------------
// tLineSegment UpdateBoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLineSegment<Tdimension, TElement>::UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const
{
  bounding_box.Add(this->Begin());
  bounding_box.Add(this->End());
}

//----------------------------------------------------------------------
// tLineSegment UpdateCenterOfGravity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLineSegment<Tdimension, TElement>::UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const
{
  center_of_gravity = 0.5 * (this->Begin() + this->End());
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
