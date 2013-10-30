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
tCardinalSplineCurve<Tdimension, TElement>::tCardinalSplineCurve() :
  tSplineCurve()
{}

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
// tCardinalSplineCurve GetSegmentForParameter
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
unsigned int tCardinalSplineCurve<Tdimension, TElement>::GetSegmentForParameter(typename tSplineCurve::tParameter t) const
{
  assert((0 <= t) && (t <= this->NumberOfSegments()));
  return static_cast<unsigned int>(t < this->NumberOfSegments() ? t : t - 1.0);
}

//----------------------------------------------------------------------
// tCardinalSplineCurve GetLocalParameter
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
typename tSplineCurve<Tdimension, TElement, 3>::tParameter tCardinalSplineCurve<Tdimension, TElement>::GetLocalParameter(typename tSplineCurve::tParameter t) const
{
  assert((0 <= t) && (t <= this->NumberOfSegments()));
  return t < this->NumberOfSegments() ? t - std::trunc(t) : 1.0;
}

//----------------------------------------------------------------------
// tCardinalSplineCurve CreateBezierCurveForSegment
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
std::shared_ptr<const typename tSplineCurve<Tdimension, TElement, 3>::tBezierCurve> tCardinalSplineCurve<Tdimension, TElement>::CreateBezierCurveForSegment(unsigned int i) const
{
  typename tShape::tPoint bezier_control_points[4];

  bezier_control_points[0] = this->ControlPoints()[i + 1];
  bezier_control_points[1] = this->ControlPoints()[i + 1] + (1.0 - this->tension) / 2.0 * (this->ControlPoints()[i + 2] - this->ControlPoints()[i]);
  bezier_control_points[2] = this->ControlPoints()[i + 2] + (1.0 - this->tension) / 2.0 * (this->ControlPoints()[i + 1] - this->ControlPoints()[i + 3]);
  bezier_control_points[3] = this->ControlPoints()[i + 2];

  return std::shared_ptr<const typename tSplineCurve::tBezierCurve>(new typename tSplineCurve::tBezierCurve(bezier_control_points, bezier_control_points + 4));
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
