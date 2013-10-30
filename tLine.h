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
/*!\file    tLine.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-27
 *
 * \brief   Contains tLine
 *
 * \b tLine
 *
 * A few words for tLine
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__tLine_h__
#define __rrlib__geometry__tLine_h__

#include "rrlib/geometry/tShape.h"
//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_
#include "rrlib/canvas/tCanvas2D.h"
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

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
// Class declaration
//----------------------------------------------------------------------
//! Short description of tLine
/*! A more detailed description of tLine, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdimension, typename TElement = double>
class tLine : public tShape<Tdimension, TElement>
{

  typedef geometry::tShape<Tdimension, TElement> tShape;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename tShape::tPoint::tElement tParameter;

  tLine();
  tLine(const typename tShape::tPoint &support, const math::tVector<Tdimension, TElement> &direction);
  tLine(const typename tShape::tPoint &support, const typename tShape::tPoint &second_support);

  inline const typename tShape::tPoint &Support() const
  {
    return this->support;
  }

  inline const math::tVector<Tdimension, TElement> &Direction() const
  {
    return this->direction;
  }

  void Set(const typename tShape::tPoint &support, const math::tVector<Tdimension, TElement> &direction);

  const typename tShape::tPoint operator()(tParameter t) const;

  const TElement GetDistanceToPoint(const typename tShape::tPoint &point) const;

  virtual const typename tShape::tPoint GetClosestPoint(const typename tShape::tPoint &reference_point) const;

  const bool GetIntersection(typename tShape::tPoint &intersection_point, const tLine &line) const;

  virtual tLine &Translate(const math::tVector<Tdimension, TElement> &translation);
  virtual tLine &Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation);
  virtual tLine &Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  typename tShape::tPoint support;
  typename math::tVector<Tdimension, TElement> direction;

  virtual void UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const;
  virtual void UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const;

};

typedef tLine<2, double> tLine2D;
typedef tLine<3, double> tLine3D;

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tLine<2, float>;
extern template class tLine<3, float>;

extern template class tLine<2, double>;
extern template class tLine<3, double>;

extern template class tLine<2, int>;
extern template class tLine<3, int>;

extern template class tLine<2, unsigned int>;
extern template class tLine<3, unsigned int>;

//----------------------------------------------------------------------
// Operators for rrlib_canvas
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_

template <typename TElement>
canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tLine<2, TElement> &line);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/tLine.hpp"

#endif
