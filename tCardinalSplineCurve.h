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
/*!\file    tCardinalSplineCurve.h
 *
 * \author  Tobias Foehst
 *
 * \date    2009-05-26
 *
 * \brief   Contains tCardinalSplineCurve
 *
 * \b tBezierCurve
 *
 * A few words for tCardinalSplineCurve
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_geometry_tCardinalSplineCurve_h_
#define _rrlib_geometry_tCardinalSplineCurve_h_

#include "rrlib/geometry/tSplineCurve.h"
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
//! Short description of tLine
/*! A more detailed description of tCardinalSplineCurve, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdimension, typename TElement>
class tCardinalSplineCurve : public tSplineCurve<Tdimension, TElement, 3>
{
  typedef geometry::tSplineCurve<Tdimension, TElement, 3> tSplineCurve;
  typedef geometry::tShape<Tdimension, TElement> tShape;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TIterator>
  tCardinalSplineCurve(TIterator begin, TIterator end, double tension = 0.0);

  template <typename TSTLContainer>
  explicit tCardinalSplineCurve(const TSTLContainer &control_points, double tension = 0.0);

  inline double GetTension() const
  {
    return this->tension;
  }

  void SetTension(double tension);

  virtual const typename tSplineCurve::tBezierCurve GetBezierCurveForSegment(unsigned int i) const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  double tension;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/tCardinalSplineCurve.hpp"

#endif
