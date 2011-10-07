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
/*!\file    tBoundingBox.h
 *
 * \author  Tobias Foehst
 *
 * \date    2009-05-27
 *
 * \brief   Contains tBoundingBox
 *
 * \b tBoundingBox
 *
 * A few words for tBoundingBox
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__tBoundingBox_h__
#define __rrlib__geometry__tBoundingBox_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tPoint.h"

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
//! Short description of tBoundingBox
/*! A more detailed description of tBoundingBox, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdimension, typename TElement>
class tBoundingBox
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef geometry::tPoint<Tdimension, TElement> tPoint;

  tBoundingBox();

  inline const tPoint &Min() const
  {
    return this->min;
  }

  inline const tPoint &Max() const
  {
    return this->max;
  }

  void Reset();

  void Add(const tPoint &point);

  void Add(const tBoundingBox &other);

  const bool Intersects(const tBoundingBox &other) const;

  const tBoundingBox GetIntersection(const tBoundingBox &other) const;

  const bool Contains(const tPoint &point) const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tPoint min;
  tPoint max;

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/tBoundingBox.hpp"

#endif
