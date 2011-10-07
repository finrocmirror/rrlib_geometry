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
template < size_t Tdimension, typename TElement = double >
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

  const TElement Length() const;

  virtual const typename tShape::tPoint GetNearestPoint(const typename tShape::tPoint &reference_point) const;

  virtual tLineSegment &Translate(const math::tVector<Tdimension, TElement> &translation);
  virtual tLineSegment &Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation);
  virtual tLineSegment &Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  typename tShape::tPoint end;

  virtual void UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const;
  virtual void UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const;

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
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/tLineSegment.hpp"

#endif
