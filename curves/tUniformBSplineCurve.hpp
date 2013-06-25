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
/*!\file    tUniformBSplineCurve.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-09-01
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
// tUniformBSplineCurve constructor
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename TIterator>
tUniformBSplineCurve<Tdimension, TElement>::tUniformBSplineCurve(TIterator begin, TIterator end, double tension)
  : tSplineCurve(begin, end),
    tension(tension)
{}

//----------------------------------------------------------------------
// tUniformBSplineCurve SetTension
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tUniformBSplineCurve<Tdimension, TElement>::SetTension(double tension)
{
  this->tension = tension;
  this->SetChanged();
}

//----------------------------------------------------------------------
// tUniformBSplineCurve GetSegmentForParameter
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
unsigned int tUniformBSplineCurve<Tdimension, TElement>::GetSegmentForParameter(typename tSplineCurve::tParameter t) const
{
  assert((0 <= t) && (t <= this->NumberOfSegments()));
  return static_cast<unsigned int>(t < this->NumberOfSegments() ? t : t - 1.0);
}

//----------------------------------------------------------------------
// tUniformBSplineCurve GetLocalParameter
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
typename tSplineCurve<Tdimension, TElement, 3>::tParameter tUniformBSplineCurve<Tdimension, TElement>::GetLocalParameter(typename tSplineCurve::tParameter t) const
{
  assert((0 <= t) && (t <= this->NumberOfSegments()));
  return t < this->NumberOfSegments() ? t - std::trunc(t) : 1.0;
}

//----------------------------------------------------------------------
// tUniformBSplineCurve CreateBezierCurveForSegment
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
std::shared_ptr<const typename tSplineCurve<Tdimension, TElement, 3>::tBezierCurve> tUniformBSplineCurve<Tdimension, TElement>::CreateBezierCurveForSegment(unsigned int i) const
{
  typename tShape::tPoint bezier_control_points[4];

  bezier_control_points[0] = (1.0 - this->tension) / 6.0 * (this->ControlPoints()[i] + this->ControlPoints()[i + 2]) + (2.0 + this->tension) / 3.0 * this->ControlPoints()[i + 1];
  bezier_control_points[1] = (2.0 + this->tension) / 3.0 * this->ControlPoints()[i + 1] + (1.0 - this->tension) / 3.0 * this->ControlPoints()[i + 2];
  bezier_control_points[2] = (1.0 - this->tension) / 3.0 * this->ControlPoints()[i + 1] + (2.0 + this->tension) / 3.0 * this->ControlPoints()[i + 2];
  bezier_control_points[3] = (1.0 - this->tension) / 6.0 * (this->ControlPoints()[i + 1] + this->ControlPoints()[i + 3]) + (2.0 + this->tension) / 3.0 * this->ControlPoints()[i + 2];

  return std::shared_ptr<const typename tSplineCurve::tBezierCurve>(new typename tSplineCurve::tBezierCurve(bezier_control_points, bezier_control_points + 4));
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
