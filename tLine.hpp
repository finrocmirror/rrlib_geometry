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
/*!\file    tLine.hpp
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
#include "rrlib/math/utilities.h"
#include "rrlib/math/tLUDecomposition.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tLineSegment.h"

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
// tLine constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLine<Tdimension, TElement>::tLine()
{
  this->direction[0] = 1;
}

template <size_t Tdimension, typename TElement>
tLine<Tdimension, TElement>::tLine(const typename tShape::tPoint &support, const math::tVector<Tdimension, TElement> &direction)
  : support(support),
    direction(direction.Normalized())
{}

template <size_t Tdimension, typename TElement>
tLine<Tdimension, TElement>::tLine(const typename tShape::tPoint &support, const typename tShape::tPoint &second_support)
  : support(support),
    direction((second_support - support).Normalized())
{}

//----------------------------------------------------------------------
// tLine Set
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLine<Tdimension, TElement>::Set(const typename tShape::tPoint &support, const math::tVector<Tdimension, TElement> &direction)
{
  this->support = support;
  this->direction = direction.Normalized();
  this->SetChanged();
}

//----------------------------------------------------------------------
// tLine Evaluation: operator ()
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tShape<Tdimension, TElement>::tPoint tLine<Tdimension, TElement>::operator()(tParameter t) const
{
  return math::tVector<Tdimension, TElement>(this->support + t * this->direction);
}

//----------------------------------------------------------------------
// tLine GetDistanceToPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const TElement tLine<Tdimension, TElement>::GetDistanceToPoint(const typename tShape::tPoint &point) const
{
  return (point - this->GetClosestPoint(point)).Length();
}

//----------------------------------------------------------------------
// tLine GetClosestPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tShape<Tdimension, TElement>::tPoint tLine<Tdimension, TElement>::GetClosestPoint(const typename tShape::tPoint &reference_point) const
{
  //          reference_point
  //                /|sin
  //               / |
  // -------support--F-----> direction
  //               cos
  math::tVector<Tdimension, TElement> support_to_reference(reference_point - this->support);
  return math::tVector<Tdimension, TElement>((*this)(EnclosedAngle(this->direction, support_to_reference).Cosine() * support_to_reference.Length()));
}

namespace
{

template <size_t Tdimension, typename TElement>
bool IntersectLineWithLine(typename tLine<Tdimension, TElement>::tPoint &intersection_point, const tLine<Tdimension, TElement> &left, const tLine<Tdimension, TElement> &right)
{
  math::tMatrix<Tdimension, 2, TElement> matrix;
  for (size_t i = 0; i < Tdimension; ++i)
  {
    matrix[i][0] = left.Direction()[i];
    matrix[i][1] = -right.Direction()[i];
  }

  try
  {
    TElement t = math::tLUDecomposition<2, TElement>(matrix).Solve(right.Support() - left.Support())[0];
    intersection_point = left.Support() + t * left.Direction();
    return true;
  }
  catch (const std::logic_error &)
  {
    return false;
  }
}

template <size_t Tdimension, typename TElement>
bool IntersectLineWithLineSegment(typename tLine<Tdimension, TElement>::tPoint &intersection_point, const tLine<Tdimension, TElement> &left, const geometry::tLineSegment<Tdimension, TElement> &right)
{
  if (!IntersectLineWithLine(intersection_point, left, right))
  {
    return false;
  }
  return math::IsEqual(right.GetDistanceToPoint(intersection_point), 0);
}

template <size_t Tdimension, typename TElement>
bool IntersectLineSegmentWithLineSegment(typename tLine<Tdimension, TElement>::tPoint &intersection_point, const tLineSegment<Tdimension, TElement> &left, const tLineSegment<Tdimension, TElement> &right)
{
  if (!left.BoundingBox().Intersects(right.BoundingBox()))
  {
    return false;
  }

  if (!IntersectLineWithLineSegment(intersection_point, left, right))
  {
    return false;
  }
  return math::IsEqual(left.GetDistanceToPoint(intersection_point), 0);
}

}

//----------------------------------------------------------------------
// tLine GetIntersection
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const bool tLine<Tdimension, TElement>::GetIntersection(typename tShape::tPoint &intersection_point, const tLine &line) const
{
  typedef geometry::tLineSegment<Tdimension, TElement> tLineSegment;
  if (const tLineSegment *this_segment = dynamic_cast<const tLineSegment *>(this))
  {
    if (const tLineSegment *line_segment = dynamic_cast<const tLineSegment *>(&line))
    {
      return IntersectLineSegmentWithLineSegment(intersection_point, *this_segment, *line_segment);
    }
    return IntersectLineWithLineSegment(intersection_point, line, *this_segment);
  }
  if (const tLineSegment *line_segment = dynamic_cast<const tLineSegment *>(&line))
  {
    return IntersectLineWithLineSegment(intersection_point, *this, *line_segment);
  }
  return IntersectLineWithLine(intersection_point, *this, line);
}

//----------------------------------------------------------------------
// tLine Translate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLine<Tdimension, TElement> &tLine<Tdimension, TElement>::Translate(const math::tVector<Tdimension, TElement> &translation)
{
  this->support += translation;
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tLine Rotate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLine<Tdimension, TElement> &tLine<Tdimension, TElement>::Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation)
{
  assert(math::IsEqual(rotation.Determinant(), 1));
  this->support = rotation * this->support;
  this->direction = rotation * this->direction;
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tLine Transform
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLine<Tdimension, TElement> &tLine<Tdimension, TElement>::Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation)
{
#ifndef NDEBUG
  for (size_t i = 0; i < Tdimension; ++i)
  {
    assert(math::IsEqual(transformation[Tdimension][i], 0));
  }
  assert(math::IsEqual(transformation[Tdimension][Tdimension], 1));
#endif
  math::tMatrix<Tdimension, Tdimension, TElement> rotation;
  for (size_t row = 0; row < Tdimension; ++row)
  {
    for (size_t column = 0; column < Tdimension; ++column)
    {
      rotation[row][column] = transformation[row][column];
    }
  }
  assert(math::IsEqual(rotation.Determinant(), 1));

  this->support = transformation.MultiplyHomogeneously(this->support);
  this->direction = rotation * this->direction;
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tLine UpdateBoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLine<Tdimension, TElement>::UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const
{}

//----------------------------------------------------------------------
// tLine UpdateCenterOfGravity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLine<Tdimension, TElement>::UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const
{
  center_of_gravity = this->GetClosestPoint(tShape::tPoint::Zero());
}

//----------------------------------------------------------------------
// Operators for rrlib_canvas
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_

template <typename TElement>
canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tLine<2, TElement> &line)
{
  canvas.DrawLine(line.Support(), line.Direction());

  return canvas;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
