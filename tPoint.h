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
/*!\file    tPoint.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-26
 *
 * \brief   Contains tPoint
 *
 * \b tPoint
 *
 * A few words for tPoint
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__tPoint_h__
#define __rrlib__geometry__tPoint_h__

#include "rrlib/math/tVector.h"
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
//! Short description of tPoint
/*! A more detailed description of tPoint, which
    Tobias Foehst hasn't done yet !!
*/
template < size_t Tdimension, typename TElement = double >
class tPoint : public math::tVector<Tdimension, TElement, math::vector::Cartesian>
{
  typedef math::tVector<Tdimension, TElement, math::vector::Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline tPoint() : tVector() {}
  inline tPoint(const tPoint &other) : tVector(*reinterpret_cast<const tVector *>(&other)) {}

  inline tPoint(const tVector &vector) : tVector(vector) {}

  template <typename ... TValues>
  explicit inline tPoint(TValues... values) : tVector(values...) {}

  inline tPoint &operator = (const tPoint &other)
  {
    return static_cast<tPoint &>(tVector::operator=(other));
  }

  inline tPoint &operator = (const tVector &vector)
  {
    return static_cast<tPoint &>(tVector::operator=(vector));
  }

  template <typename TOtherElement>
  inline tPoint &operator = (const math::tVector<Tdimension, TOtherElement> &other)
  {
    return static_cast<tPoint &>(tVector::operator=(other));
  }

};

typedef tPoint<2, double> tPoint2D;
typedef tPoint<3, double> tPoint3D;

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tPoint<2, float>;
extern template class tPoint<3, float>;

extern template class tPoint<2, double>;
extern template class tPoint<3, double>;

extern template class tPoint<2, int>;
extern template class tPoint<3, int>;

extern template class tPoint<2, unsigned int>;
extern template class tPoint<3, unsigned int>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
