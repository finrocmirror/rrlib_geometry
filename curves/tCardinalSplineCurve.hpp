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
/*!\file    tCardinalSplineCurve.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2009-05-26
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
// tCardinalSplineCurve constructor
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename TIterator>
tCardinalSplineCurve<Tdimension, TElement>::tCardinalSplineCurve(TIterator begin, TIterator end, double tension)
    : tSplineCurve(begin, end),
    tension(tension)
{}

//----------------------------------------------------------------------
// tCardinalSplineCurve SetTension
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tCardinalSplineCurve<Tdimension, TElement>::SetTension(double tension)
{
  this->tension = tension;
  this->SetChanged();
}

//----------------------------------------------------------------------
// tCardinalSplineCurve GetBezierCurveForSegment
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tSplineCurve<Tdimension, TElement, 3>::tBezierCurve tCardinalSplineCurve<Tdimension, TElement>::GetBezierCurveForSegment(unsigned int i) const
{
  typename tShape::tPoint bezier_control_points[4];

  bezier_control_points[0] = this->ControlPoints()[i + 1];
  bezier_control_points[1] = this->ControlPoints()[i + 1] + (1.0 - this->tension) / 2.0 * (this->ControlPoints()[i + 2] - this->ControlPoints()[i]);
  bezier_control_points[2] = this->ControlPoints()[i + 2] + (1.0 - this->tension) / 2.0 * (this->ControlPoints()[i + 1] - this->ControlPoints()[i + 3]);
  bezier_control_points[3] = this->ControlPoints()[i + 2];

  return typename tSplineCurve::tBezierCurve(bezier_control_points, bezier_control_points + 4);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
