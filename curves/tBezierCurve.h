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
/*!\file    tBezierCurve.h
 *
 * \author  Tobias Foehst
 *
 * \date    2009-05-25
 *
 * \brief   Contains tBezierCurve
 *
 * \b tBezierCurve
 *
 * A few words for tBezierCurve
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__curves__tBezierCurve_h__
#define __rrlib__geometry__curves__tBezierCurve_h__

#include "rrlib/geometry/tShape.h"
//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>

#ifdef _LIB_RRLIB_CANVAS_PRESENT_
#include "rrlib/canvas/tCanvas2D.h"
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tLine.h"

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
//! Short description of tBezierCurve
/*! A more detailed description of tBezierCurve, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
class tBezierCurve : public tShape<Tdimension, TElement>
{
  friend class tBezierCurve < Tdimension, TElement, Tdegree + 1 >;

  typedef geometry::tShape<Tdimension, TElement> tShape;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static inline const unsigned int Degree()
  {
    return Tdegree;
  }

  static inline const unsigned int NumberOfControlPoints()
  {
    return Tdegree + 1;
  }

  typedef typename tShape::tPoint::tElement tParameter;

  typedef tBezierCurve < Tdimension, TElement, Tdegree - 1 > tDerivative;
  typedef std::pair<tBezierCurve, tBezierCurve> tSubdivision;

  template <typename TIterator>
  tBezierCurve(TIterator begin, TIterator end);

  inline const typename tShape::tPoint *ControlPoints() const
  {
    return this->control_points;
  }

  void SetControlPoint(size_t i, const typename tShape::tPoint &point);

  const typename tShape::tPoint operator()(tParameter t) const;

  const TElement GetTwist() const;

  const tSubdivision GetSubdivision() const;

  const tDerivative GetDerivative() const;

  template <unsigned int Tother_degree>
  const bool GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
                              const tBezierCurve<Tdimension, TElement, Tother_degree> &other) const;

  const bool GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
                              const tLine<Tdimension, TElement> &line) const;

  const typename tShape::tPoint GetClosestPoint(const typename tShape::tPoint &reference_point) const;

  virtual tBezierCurve &Translate(const math::tVector<Tdimension, TElement> &translation);
  virtual tBezierCurve &Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation);
  virtual tBezierCurve &Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  typename tShape::tPoint control_points[Tdegree + 1];

  virtual void UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const;
  virtual void UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const;

  template <unsigned int Tother_degree>
  const bool GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
                              const tBezierCurve<Tdimension, TElement, Tother_degree> &other,
                              tParameter min_parameter, tParameter max_parameter) const;

  const bool GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
                              const tLine<Tdimension, TElement> &line,
                              tParameter min_parameter, tParameter max_parameter) const;

};

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tBezierCurve<2, float, 2>;
extern template class tBezierCurve<2, float, 3>;

extern template class tBezierCurve<3, float, 2>;
extern template class tBezierCurve<3, float, 3>;

extern template class tBezierCurve<2, double, 2>;
extern template class tBezierCurve<2, double, 3>;

extern template class tBezierCurve<3, double, 2>;
extern template class tBezierCurve<3, double, 3>;

//----------------------------------------------------------------------
// Operators for rrlib_canvas
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_

template <typename TElement>
canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tBezierCurve<2, TElement, 2> &bezier_curve);

template <typename TElement>
canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tBezierCurve<2, TElement, 3> &bezier_curve);

template <typename TElement, unsigned int Tdegree>
canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tBezierCurve<2, TElement, Tdegree> &bezier_curve);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/curves/tBezierCurve.hpp"

#endif
