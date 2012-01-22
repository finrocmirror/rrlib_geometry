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
/*!\file    tSplineCurve.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-09-01
 *
 * \brief   Contains tSplineCurve
 *
 * \b tSplineCurve
 *
 * A few words for tSplineCurve
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__curves__tSplineCurve_h__
#define __rrlib__geometry__curves__tSplineCurve_h__

#include "rrlib/geometry/tShape.h"
//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_
#include "rrlib/canvas/tCanvas2D.h"
#include "rrlib/canvas/tCanvas3D.h"
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/curves/tBezierCurve.h"

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
/*! A more detailed description of tLine, which
    Tobias Foehst hasn't done yet !!
*/
template < size_t Tdimension, typename TElement, unsigned int Tdegree = 3 >
class tSplineCurve : public tShape<Tdimension, TElement>
{

  typedef geometry::tShape<Tdimension, TElement> tShape;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static inline const unsigned int Degree()
  {
    return Tdegree;
  }

  typedef geometry::tBezierCurve<Tdimension, TElement, Tdegree> tBezierCurve;
  typedef typename tBezierCurve::tParameter tParameter;

  template <typename TIterator>
  tSplineCurve(TIterator begin, TIterator end);

  inline const size_t NumberOfControlPoints() const
  {
    return this->control_points.size();
  }

  inline const unsigned int NumberOfSegments() const
  {
    return this->control_points.size() - Tdegree;
  };

  inline const std::vector<typename tShape::tPoint> &ControlPoints() const
  {
    return this->control_points;
  }

  void SetControlPoint(size_t i, const typename tShape::tPoint &point);

  void AppendControlPoint(const typename tShape::tPoint &point);

  void InsertControlPoint(typename std::vector<typename tShape::tPoint>::iterator position, const typename tShape::tPoint &point);

  const typename tShape::tPoint operator()(tParameter t) const;

  const tBezierCurve GetBezierCurveForParameter(tParameter t) const;

  const tBezierCurve GetBezierCurveForParameter(tParameter t, tParameter &local_t) const;

  virtual const tBezierCurve GetBezierCurveForSegment(unsigned int i) const = 0;

  template <unsigned int Tother_degree>
  void GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
                        const geometry::tSplineCurve<Tdimension, TElement, Tother_degree> &other_spline) const;

  template <unsigned int Tother_degree>
  void GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
                        const geometry::tBezierCurve<Tdimension, TElement, Tother_degree> &bezier_curve) const;

  void GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
                        const tLine<Tdimension, TElement> &line) const;

  const typename tShape::tPoint GetClosestPoint(const typename tShape::tPoint &reference_point) const;

  virtual tSplineCurve &Translate(const math::tVector<Tdimension, TElement> &translation);
  virtual tSplineCurve &Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation);
  virtual tSplineCurve &Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  std::vector<typename tShape::tPoint> control_points;

  virtual void UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const;
  virtual void UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const;

};

//----------------------------------------------------------------------
// Operators for rrlib_canvas
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_

template <typename TElement>
inline canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tSplineCurve<2, TElement, 3> &spline);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/geometry/curves/tSplineCurve.hpp"

#endif
