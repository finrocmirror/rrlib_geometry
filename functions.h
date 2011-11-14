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
/*!\file    functions.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-27
 *
 * \brief
 *
 * \b
 *
 * A few words for functions.h
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__functions_h__
#define __rrlib__geometry__functions_h__

//----------------------------------------------------------------------
// External includes with <>
//----------------------------------------------------------------------
#include <cmath>

#include <boost/utility/enable_if.hpp>

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



template <typename TElement>
inline const math::tAngleRad GetSlope(const tLineSegment<2, TElement> &line_segment)
{
  return std::atan2(line_segment.End().Y() - line_segment.Begin().Y(), line_segment.End().X() - line_segment.Begin().X());
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
inline const typename boost::enable_if_c<(Tdimension <= 3), double>::type GetCurvature(const tBezierCurve<Tdimension, TElement, Tdegree> &curve, TElement parameter)
{
  typename tBezierCurve<Tdimension, TElement, Tdegree>::tDerivative first_derivative(curve.GetDerivative());
  typename tBezierCurve<Tdimension, TElement, Tdegree>::tDerivative::tDerivative second_derivative(first_derivative.GetDerivative());

  math::tVector<3, TElement, math::vector::Cartesian> first(first_derivative(parameter));
  math::tVector<3, TElement, math::vector::Cartesian> second(second_derivative(parameter));
  double first_length = first.Length();

  return CrossProduct(first, second).Length() / (first_length * first_length * first_length);
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
