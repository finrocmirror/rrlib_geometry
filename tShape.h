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
/*!\file    tShape.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-27
 *
 * \brief   Contains tShape
 *
 * \b tShape
 *
 * A few words for tShape
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__tShape_h__
#define __rrlib__geometry__tShape_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
#include "rrlib/math/tMatrix.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tPoint.h"
#include "rrlib/geometry/tBoundingBox.h"

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
//! Short description of tShape
/*! A more detailed description of tShape, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdimension, typename TElement>
class tShape
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static inline const size_t Dimension()
  {
    return Tdimension;
  }

  typedef geometry::tPoint<Tdimension, TElement> tPoint;
  typedef geometry::tBoundingBox<Tdimension, TElement> tBoundingBox;

  tShape();

  virtual ~tShape() = 0;

  const tBoundingBox &BoundingBox() const;
  const tPoint &CenterOfGravity() const;

  virtual tShape &Translate(const math::tVector<Tdimension, TElement> &translation) = 0;
  virtual tShape &Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation) = 0;
  virtual tShape &Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation) = 0;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  virtual void SetChanged();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  mutable bool changed;
  mutable tBoundingBox *bounding_box;
  mutable tPoint *center_of_gravity;

  void ResetMetaInformation() const;

  virtual void UpdateBoundingBox(tBoundingBox &bounding_box) const = 0;
  virtual void UpdateCenterOfGravity(tPoint &center_of_gravity) const = 0;

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/tShape.hpp"

#endif
