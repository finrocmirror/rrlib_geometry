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
/*!\file    tLineSegment.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-27
 *
 * \brief   Contains tLineSegment
 *
 * \b tLineSegment
 *
 * A few words for tLineSegment
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__tLineSegment_h__
#define __rrlib__geometry__tLineSegment_h__

#ifndef __rrlib__geometry__tLine_h__
#error Invalid include directive. Try #include "rrlib/geometry/tLine.h" instead.
#endif

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
//! Short description of tLineSegment
/*! A more detailed description of tLineSegment, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdimension, typename TElement = double>
class tLineSegment : public tLine<Tdimension, TElement>
{

  typedef geometry::tLine<Tdimension, TElement> tLine;
  typedef geometry::tShape<Tdimension, TElement> tShape;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tLineSegment();
  tLineSegment(const typename tShape::tPoint &begin, const typename tShape::tPoint &end);

  inline const typename tShape::tPoint &Begin() const
  {
    return this->Support();
  }
  inline const typename tShape::tPoint &End() const
  {
    return this->end;
  }

  void Set(const typename tShape::tPoint &begin, const typename tShape::tPoint &end);

  inline const typename tShape::tPoint &MidPoint()
  {
    return this->CenterOfGravity();
  }

  inline const TElement Length() const
  {
    return (this->Begin() - this->End()).Length();
  }

  virtual const typename tShape::tPoint GetClosestPoint(const typename tShape::tPoint &reference_point) const override;

  virtual tLineSegment &Translate(const math::tVector<Tdimension, TElement> &translation) override;
  virtual tLineSegment &Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation) override;
  virtual tLineSegment &Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation) override;

  /*!
   * Calculates intersection of this line segment with bounding box.
   *
   * \param bounding_box Bounding box for intersection
   * \return 'first' contains whether line intersects bounding box; 'second' the intersection (has the same direction as this line segment)
   */
  virtual std::pair<bool, tLineSegment> GetIntersection(typename tShape::tBoundingBox &bounding_box) const override;
  using tLine::GetIntersection;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  typename tShape::tPoint end;

  virtual void UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const override;
  virtual void UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const override;

};

typedef tLineSegment<2, double> tLineSegment2D;
typedef tLineSegment<3, double> tLineSegment3D;

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tLineSegment<2, float>;
extern template class tLineSegment<3, float>;

extern template class tLineSegment<2, double>;
extern template class tLineSegment<3, double>;

extern template class tLineSegment<2, int>;
extern template class tLineSegment<3, int>;

extern template class tLineSegment<2, unsigned int>;
extern template class tLineSegment<3, unsigned int>;

//----------------------------------------------------------------------
// Operators for rrlib_canvas
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_

template <typename TElement>
canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tLineSegment<2, TElement> &line);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/tLineSegment.hpp"

#endif
