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
/*!\file    tBSplineCurve.h
 *
 * \author  Patrick Fleischmann
 *
 * \date    Apr 18, 2012
 *
 * \brief
 *
 * \b tBSplineCurve.h
 *
 *
 *
 */
//----------------------------------------------------------------------
#ifndef TBSPLINECURVE_H_
#define TBSPLINECURVE_H_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tPoint.h"
#include "rrlib/geometry/curves/tSplineCurve.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
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
template < size_t Tdimension, typename TElement, unsigned int Tdegree = 3 >
class tBSplineCurve : public tSplineCurve<Tdimension, TElement, Tdegree>
{
  typedef geometry::tSplineCurve<Tdimension, TElement, Tdegree> tSplineCurve;
  typedef geometry::tShape<Tdimension, TElement> tShape;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tBSplineCurve();

  /*!
     * Construct a uniform B-spline using control points and a knot vector
     */
  template <typename TIterator, typename TKnotIterator>
  tBSplineCurve(TIterator control_points_begin, TIterator control_points_end, TKnotIterator knots_begin, TKnotIterator knots_end);

  /*!
   * Construct a uniform B-spline using control points. The (uniform knot) vector is calculated automatically.
   */
  template <typename TIterator>
  tBSplineCurve(TIterator begin, TIterator end);

  const std::vector<double> &Knots() const
  {
    return this->knots;
  }

  template <typename TIterator>
  void SetKnots(TIterator begin, TIterator end)
  {
    this->knots.assign(begin, end);
    this->CalculateBezierControlPoints();
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:
  /*!
   * Method is called in control point insertion, appending and modification. Causes knot vector and bezier control point recalculation
   */
  virtual void SetChanged();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:
  /*! B-spline knot vector (#knots = #control points + degree + 1) */
  std::vector<double> knots;
  /*! Bezier control points used to evaluate b-spline curve which is converted to a set of bezier curves  */
  std::vector<typename tShape::tPoint> bezier_control_points;

  /*!
   * Calculate a uniform knot vector, this->knots will be modified
   */
  void CalculateKnotVector();

  /*!
   * Calculate all Bezier control points by knot insertion, this->bezier_control_points will be modified
   */
  void CalculateBezierControlPoints();

  /*!
   * Recalculate control points affected by a knot insertion
   * @param at Index of the knot to be inserted
   * @param knots_before_insertion Knot vector before knot insertion
   * @param knot The new knot that should be inserted
   * @param control_points Control points before insertion
   * @return Control points after knot insertion
   */
  std::vector<typename tShape::tPoint> InsertKnot(int at, const std::vector<double> &knots_before_insertion, double knot, const std::vector<typename tShape::tPoint> &control_points) const;

  virtual unsigned int GetSegmentForParameter(typename tSplineCurve::tParameter t);
  virtual typename tSplineCurve::tParameter GetLocalParameter(typename tSplineCurve::tParameter t);

  /*!
   * Create the bezier representation of the B-spline for the given segment id
   * @param id ID of the segment
   * @return Bezier curve of the segment
   */
  virtual std::shared_ptr<const typename tSplineCurve::tBezierCurve> CreateBezierCurveForSegment(unsigned int i) const;

};

//----------------------------------------------------------------------
// Operators for rrlib_serialization
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tBSplineCurve<Tdimension, TElement, Tdegree> &spline);

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tBSplineCurve<Tdimension, TElement, Tdegree> &spline);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/curves/tBSplineCurve.hpp"

#endif
