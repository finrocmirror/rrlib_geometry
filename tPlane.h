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
/*!\file    tPlane.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-19
 *
 * \brief   Contains tPlane
 *
 * \b tPlane
 *
 * A few words for tPlane
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__tPlane_h__
#define __rrlib__geometry__tPlane_h__

#include "rrlib/geometry/tShape.h"
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
//! Short description of tPlane
/*! A more detailed description of tPlane, which
    Tobias Foehst hasn't done yet !!
*/
template < size_t Tdimension, typename TElement = double >
class tPlane : public tShape<3, TElement>
{

  typedef geometry::tShape<Tdimension, TElement> tShape;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename tShape::tPoint::tElement tParameter;

  tPlane();
  tPlane(const typename tShape::tPoint &support, const math::tVector<Tdimension, TElement> &normal);
  tPlane(const typename tShape::tPoint &p1, const typename tShape::tPoint &p2, const typename tShape::tPoint &p3);

  inline const typename tShape::tPoint &Support() const
  {
    return this->support;
  }

  inline const math::tVector<Tdimension, TElement> &Normal() const
  {
    return this->normal;
  }

  void Set(const typename tShape::tPoint &support, const math::tVector<Tdimension, TElement> &normal);
  void Set(const typename tShape::tPoint &p1, const typename tShape::tPoint &p2, const typename tShape::tPoint &p3);

//  const typename tShape::tPoint operator()(tParameter t) const;

  const TElement GetDistanceToPoint(const typename tShape::tPoint &point) const;

  virtual const typename tShape::tPoint GetNearestPoint(const typename tShape::tPoint &reference_point) const;

  virtual tPlane &Translate(const math::tVector<Tdimension, TElement> &translation);
  virtual tPlane &Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation);
  virtual tPlane &Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  typename tShape::tPoint support;
  typename math::tVector<Tdimension, TElement> normal;

  virtual void UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const;
  virtual void UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const;

};

typedef tPlane<3, double> tPlane3D;

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tPlane<3, float>;
extern template class tPlane<3, double>;



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/tPlane.hpp"

#endif
