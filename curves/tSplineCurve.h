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

  static const unsigned int cDEGREE;

  typedef geometry::tBezierCurve<Tdimension, TElement, Tdegree> tBezierCurve;
  typedef typename tBezierCurve::tParameter tParameter;

  template <typename TIterator>
  tSplineCurve(TIterator begin, TIterator end);

  template <typename TSTLContainer>
  explicit tSplineCurve(const TSTLContainer &control_points);

  inline const size_t GetNumberOfControlPoints() const
  {
    return this->control_points.size();
  }

  inline const std::vector<typename tShape::tPoint> &GetControlPoints() const
  {
    return this->control_points;
  }

  inline const typename tShape::tPoint &GetControlPoint(size_t i) const
  {
    return this->control_points[i];
  }

  void SetControlPoint(size_t i, const typename tShape::tPoint &point);

  void AppendControlPoint(const typename tShape::tPoint &point);

  void InsertControlPoint(typename std::vector<typename tShape::tPoint>::iterator position, const typename tShape::tPoint &point);

  const typename tShape::tPoint operator()(tParameter t) const;

  const unsigned int GetNumberOfSegments() const;

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
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/curves/tSplineCurve.hpp"

#endif
