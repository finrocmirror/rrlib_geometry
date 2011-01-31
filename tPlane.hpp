//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    tPlane.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-19
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"

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
// tPlane constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tPlane<Tdimension, TElement>::tPlane()
{
  this->normal[0] = 1;
}

template <size_t Tdimension, typename TElement>
tPlane<Tdimension, TElement>::tPlane(const typename tShape::tPoint &support, const math::tVector<Tdimension, TElement> &normal)
    : support(support),
    normal(normal.Normalized())
{}

template <size_t Tdimension, typename TElement>
tPlane<Tdimension, TElement>::tPlane(const typename tShape::tPoint &p1, const typename tShape::tPoint &p2, const typename tShape::tPoint &p3)
    : support(p1),
    normal(CrossProduct(p2 - p1, p3 - p1).Normalized())
{}

//----------------------------------------------------------------------
// tPlane Set
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tPlane<Tdimension, TElement>::Set(const typename tShape::tPoint &support, const math::tVector<Tdimension, TElement> &direction)
{
  this->support = support;
  this->normal = normal.Normalized();
  this->SetChanged();
}

template <size_t Tdimension, typename TElement>
void tPlane<Tdimension, TElement>::Set(const typename tShape::tPoint &p1, const typename tShape::tPoint &p2, const typename tShape::tPoint &p3)
{
  this->support = p1;
  this->normal = CrossProduct(p2 - p1, p3 - p1).Normalized();
  this->SetChanged();
}

//----------------------------------------------------------------------
// tPlane Evaluation: operator ()
//----------------------------------------------------------------------
//template <size_t Tdimension, typename TElement>
//const typename tShape<Tdimension, TElement>::tPoint tPlane<Tdimension, TElement>::operator()(tParameter t) const
//{
//  return math::tVector<Tdimension, TElement>(this->support + t * this->direction);
//}

//----------------------------------------------------------------------
// tPlane GetDistanceToPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const TElement tPlane<Tdimension, TElement>::GetDistanceToPoint(const typename tShape::tPoint &point) const
{
  return math::AbsoluteValue(this->normal *(point - this->support));
}

//----------------------------------------------------------------------
// tPlane GetNearestPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tShape<Tdimension, TElement>::tPoint tPlane<Tdimension, TElement>::GetNearestPoint(const typename tShape::tPoint &reference_point) const
{
  return -(this->normal *(reference_point - this->support)) * this->normal + reference_point;
}

//----------------------------------------------------------------------
// tPlane Translate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tPlane<Tdimension, TElement> &tPlane<Tdimension, TElement>::Translate(const math::tVector<Tdimension, TElement> &translation)
{
  this->support += translation;
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tPlane Rotate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tPlane<Tdimension, TElement> &tPlane<Tdimension, TElement>::Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation)
{
  assert(math::IsEqual(rotation.Determinant(), 0));
  this->support = rotation * this->support;
  this->normal = rotation * this->normal;
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tPlane Transform
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tPlane<Tdimension, TElement> &tPlane<Tdimension, TElement>::Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation)
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
  assert(math::IsEqual(rotation.Determinant(), 0));

  this->support = transformation.MultiplyHomogeneously(this->support);
  this->normal = rotation * this->normal;
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tPlane UpdateBoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tPlane<Tdimension, TElement>::UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const
{}

//----------------------------------------------------------------------
// tPlane UpdateCenterOfGravity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tPlane<Tdimension, TElement>::UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const
{
  center_of_gravity = this->GetNearestPoint(tShape::tPoint::Zero());
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
