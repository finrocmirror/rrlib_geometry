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
/*!\file    tBezierCurve.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2009-05-25
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/variadic_templates.h"

#include <iomanip>
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
// tBezierCurve constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
tBezierCurve<Tdimension, TElement, Tdegree>::tBezierCurve()
{
  static_assert(Tdegree > 0, "The degree of Bezier curves must be greater than zero");
  memset(this->control_points, 0, sizeof(this->control_points));
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <typename TIterator>
tBezierCurve<Tdimension, TElement, Tdegree>::tBezierCurve(TIterator begin, TIterator end)
{
  static_assert(Tdegree > 0, "The degree of Bezier curves must be greater than zero");
  assert(static_cast<size_t>(std::distance(begin, end)) == this->NumberOfControlPoints() && "A Bezier curve must have degree + 1 control points");
  size_t index = 0;
  for (TIterator it = begin; it != end; ++it)
  {
    this->control_points[index++] = *it;
  }
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <typename ... TPoints>
tBezierCurve<Tdimension, TElement, Tdegree>::tBezierCurve(const typename tShape::tPoint &p1, const typename tShape::tPoint &p2, const TPoints &... rest)
{
  static_assert(Tdegree > 0, "The degree of Bezier curves must be greater than zero");
  static_assert(sizeof...(rest) + 1 == Tdegree, "A Bezier curve must have degree + 1 control points");
  size_t index = 0;
  util::ProcessVariadicValues([this, &index](const typename tShape::tPoint & x)
  {
    this->control_points[index++] = x;
  },
  p1, p2, rest...);
}

//----------------------------------------------------------------------
// tBezierCurve SetControlPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tBezierCurve<Tdimension, TElement, Tdegree>::SetControlPoint(size_t i, const typename tShape::tPoint &point)
{
  assert(i < this->NumberOfControlPoints());
  this->control_points[i] = point;
  this->SetChanged();
};

//----------------------------------------------------------------------
// tBezierCurve GetTwist
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const TElement tBezierCurve<Tdimension, TElement, Tdegree>::GetTwist() const
{
  TElement twist = 0.0;
  for (size_t i = 0; i < Tdegree - 1; ++i)
  {
    twist = std::max(twist, ((this->control_points[i + 2] - this->control_points[i + 1]) - (this->control_points[i + 1] - this->control_points[i])).Length());
  }
  return Tdegree * (Tdegree - 1) * twist;
}

//----------------------------------------------------------------------
// tBezierCurve GetSubdivision
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tBezierCurve<Tdimension, TElement, Tdegree>::tSubdivision tBezierCurve<Tdimension, TElement, Tdegree>::GetSubdivision() const
{
  const size_t cNUMBER_OF_CONTROL_POINTS = Tdegree + 1;
  typename tShape::tPoint left_half[cNUMBER_OF_CONTROL_POINTS];
  typename tShape::tPoint right_half[cNUMBER_OF_CONTROL_POINTS];
  typename tShape::tPoint temp_points[cNUMBER_OF_CONTROL_POINTS];
  std::memcpy(temp_points, this->control_points, cNUMBER_OF_CONTROL_POINTS * sizeof(typename tShape::tPoint));

  left_half[0] = temp_points[0];
  right_half[Tdegree] = temp_points[Tdegree];

  size_t k = 0;
  while (k < Tdegree)
  {
    for (size_t i = 0; i < Tdegree - k; ++i)
    {
      temp_points[i] = (temp_points[i] + temp_points[i + 1]) * 0.5;
    }
    ++k;
    left_half[k] = temp_points[0];
    right_half[Tdegree - k] = temp_points[Tdegree - k];
  }

  return std::make_pair(tBezierCurve(left_half, left_half + cNUMBER_OF_CONTROL_POINTS), tBezierCurve(right_half, right_half + cNUMBER_OF_CONTROL_POINTS));
}

//----------------------------------------------------------------------
// tBezierCurve Evaluation: operator ()
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tShape<Tdimension, TElement>::tPoint tBezierCurve<Tdimension, TElement, Tdegree>::operator()(tParameter t) const
{
  assert((0.0 <= t) && (t <= 1.0));

  const size_t cNUMBER_OF_CONTROL_POINTS = Tdegree + 1;
  typename tShape::tPoint temp_points[cNUMBER_OF_CONTROL_POINTS];
  std::memcpy(temp_points, this->control_points, cNUMBER_OF_CONTROL_POINTS * sizeof(typename tShape::tPoint));

  size_t k = 0;
  while (k < Tdegree)
  {
    for (size_t i = 0; i < Tdegree - k; ++i)
    {
      temp_points[i] = ((1.0 - t) * temp_points[i]) + (t * temp_points[i + 1]);
    }
    ++k;
  }

  return temp_points[0];
}

//----------------------------------------------------------------------
// tBezierCurve GetDerivative
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tBezierCurve<Tdimension, TElement, Tdegree>::tDerivative tBezierCurve<Tdimension, TElement, Tdegree>::GetDerivative() const
{
  typename tShape::tPoint temp[Tdegree];
  for (size_t i = 0; i < Tdegree; ++i)
  {
    temp[i] = Tdegree * (this->control_points[i + 1] - this->control_points[i]);
  }
  return tDerivative(temp, temp + Tdegree);
}

//----------------------------------------------------------------------
// tBezierCurve GetIntersections
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <unsigned int Tother_degree>
const bool tBezierCurve<Tdimension, TElement, Tdegree>::GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
    const tBezierCurve<Tdimension, TElement, Tother_degree> &other) const
{
  return this->GetIntersections(intersection_points, intersection_parameters, other, 0.0, 1.0);
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const bool tBezierCurve<Tdimension, TElement, Tdegree>::GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
    const tLine<Tdimension, TElement> &line) const
{
  return this->GetIntersections(intersection_points, intersection_parameters, line, 0.0, 1.0);
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <unsigned int Tother_degree>
const bool tBezierCurve<Tdimension, TElement, Tdegree>::GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
    const tBezierCurve<Tdimension, TElement, Tother_degree> &other,
    tParameter min_parameter, tParameter max_parameter) const
{
  // check if bounding boxes intersect
  if (!this->BoundingBox().Intersects(other.BoundingBox()))
  {
    return false;
  }

  // subdivide curve with greater twist if necessary
  TElement own_twist = this->GetTwist();
  TElement other_twist = other.GetTwist();

  if (own_twist > other_twist && !math::IsEqual(own_twist, 0))
  {
    tSubdivision subdivision(this->GetSubdivision());
    tParameter middle_parameter(0.5 * (min_parameter + max_parameter));
    bool result = false;
    result |= subdivision.first.GetIntersections(intersection_points, intersection_parameters, other, min_parameter, middle_parameter);
    result |= subdivision.second.GetIntersections(intersection_points, intersection_parameters, other, middle_parameter, max_parameter);
    return result;
  }
  else if (!math::IsEqual(other_twist, 0))
  {
    tSubdivision subdivision(other.GetSubdivision());
    bool result = false;
    result |= this->GetIntersections(intersection_points, intersection_parameters, subdivision.first, min_parameter, max_parameter);
    result |= this->GetIntersections(intersection_points, intersection_parameters, subdivision.second, min_parameter, max_parameter);
    return result;
  }

  // compute the intersection using the baselines of the two control polygons
  tLine<Tdimension, TElement> own_line_segment(this->control_points[0], this->control_points[Tdegree]);
  tLine<Tdimension, TElement> other_line_segment(other.ControlPoints()[0], other.ControlPoints()[Tother_degree]);
  typename tShape::tPoint intersection_point;
  if (own_line_segment.GetIntersection(intersection_point, other_line_segment) && !math::IsEqual(intersection_point, intersection_points.back(), 0.001))
  {
    intersection_points.push_back(intersection_point);
    const auto baseline_parameter = math::LimitedValue<tParameter>((intersection_points.back() - this->control_points[0]).Length() / (this->control_points[Tdegree] - this->control_points[0]).Length(), 0, 1);
    assert(!std::isnan(baseline_parameter));
    intersection_parameters.push_back(min_parameter + (max_parameter - min_parameter) * baseline_parameter);
    return true;
  }
  return false;
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const bool tBezierCurve<Tdimension, TElement, Tdegree>::GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
    const tLine<Tdimension, TElement> &line,
    tParameter min_parameter, tParameter max_parameter) const
{
  // check if intersection with line is possible
  typename tShape::tPoint center_of_bounding_box(math::tVector<Tdimension, TElement>(0.5 * (this->BoundingBox().Min() + this->BoundingBox().Max())));
  double radius_of_bounding_sphere = 0.5 * (this->BoundingBox().Max() - this->BoundingBox().Min()).Length();
  if (line.GetDistanceToPoint(center_of_bounding_box) > radius_of_bounding_sphere)
  {
    return false;
  }

  // subdivide curve if necessary
  if (!math::IsEqual(this->GetTwist(), 0))
  {
    tSubdivision subdivision(this->GetSubdivision());
    tParameter middle_parameter(0.5 * (min_parameter + max_parameter));
    bool result = false;
    result |= subdivision.first.GetIntersections(intersection_points, intersection_parameters, line, min_parameter, middle_parameter);
    result |= subdivision.second.GetIntersections(intersection_points, intersection_parameters, line, middle_parameter, max_parameter);
    return result;
  }

  // compute the intersection using the baseline of the control polygon
  tLineSegment<Tdimension, TElement> own_line_segment(this->control_points[0], this->control_points[Tdegree]);
  typename tShape::tPoint intersection_point;
  if (own_line_segment.GetIntersection(intersection_point, line))
  {
    intersection_points.push_back(intersection_point);
    const auto baseline_parameter = math::LimitedValue<tParameter>((intersection_points.back() - this->control_points[0]).Length() / (this->control_points[Tdegree] - this->control_points[0]).Length(), 0, 1);
    assert(!std::isnan(baseline_parameter));
    intersection_parameters.push_back(min_parameter + (max_parameter - min_parameter) * baseline_parameter);
    return true;
  }
  return false;
}

//----------------------------------------------------------------------
// tBezierCurve GetClosestPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tShape<Tdimension, TElement>::tPoint tBezierCurve<Tdimension, TElement, Tdegree>::GetClosestPoint(const typename tShape::tPoint &reference_point) const
{
  tSubdivision subdivision(this->GetSubdivision());

  double squared_distance_to_first_candidate = std::numeric_limits<double>::max();
  double squared_distance_to_second_candidate = std::numeric_limits<double>::max();

  for (unsigned int i = 0; i < Tdegree; ++i)
  {
    tLineSegment<Tdimension, TElement> first_segment_base_line(subdivision.first.ControlPoints()[i], subdivision.first.ControlPoints()[i + 1]);
    tLineSegment<Tdimension, TElement> second_segment_base_line(subdivision.second.ControlPoints()[i], subdivision.second.ControlPoints()[i + 1]);

    typename tShape::tPoint first_candidate = first_segment_base_line.GetClosestPoint(reference_point);
    typename tShape::tPoint second_candidate = second_segment_base_line.GetClosestPoint(reference_point);

    double squared_distance_to_first = (reference_point - first_candidate).SquaredLength();
    double squared_distance_to_second = (reference_point - second_candidate).SquaredLength();

    if (squared_distance_to_first < squared_distance_to_first_candidate)
    {
      squared_distance_to_first_candidate = squared_distance_to_first;
    }

    if (squared_distance_to_second < squared_distance_to_second_candidate)
    {
      squared_distance_to_second_candidate = squared_distance_to_second;
    }
  }

  if (squared_distance_to_first_candidate < squared_distance_to_second_candidate)
  {
    if (subdivision.first.GetTwist() < 1E-6)
    {
      tLineSegment<Tdimension, TElement> first_segment_base_line(subdivision.first.ControlPoints()[0], subdivision.first.ControlPoints()[Tdegree]);
      return first_segment_base_line.GetClosestPoint(reference_point);
    }
    return subdivision.first.GetClosestPoint(reference_point);
  }

  if (subdivision.second.GetTwist() < 1E-6)
  {
    tLineSegment<Tdimension, TElement> second_segment_base_line(subdivision.second.ControlPoints()[0], subdivision.second.ControlPoints()[Tdegree]);
    return second_segment_base_line.GetClosestPoint(reference_point);
  }
  return subdivision.second.GetClosestPoint(reference_point);
}

//----------------------------------------------------------------------
// tBezierCurve Translate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
tBezierCurve<Tdimension, TElement, Tdegree> &tBezierCurve<Tdimension, TElement, Tdegree>::Translate(const math::tVector<Tdimension, TElement> &translation)
{
  for (size_t i = 0; i < this->NumberOfControlPoints(); ++i)
  {
    this->control_points[i] += translation;
  }
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tBezierCurve Rotate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
tBezierCurve<Tdimension, TElement, Tdegree> &tBezierCurve<Tdimension, TElement, Tdegree>::Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation)
{
  for (size_t i = 0; i < this->NumberOfControlPoints(); ++i)
  {
    this->control_points[i] = rotation * this->control_points[i];
  }
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tBezierCurve Transform
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
tBezierCurve<Tdimension, TElement, Tdegree> &tBezierCurve<Tdimension, TElement, Tdegree>::Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation)
{
#ifndef NDEBUG
  for (size_t i = 0; i < Tdimension; ++i)
  {
    assert(math::IsEqual(transformation[Tdimension][i], 0));
  }
  assert(math::IsEqual(transformation[Tdimension][Tdimension], 1));
  math::tMatrix<Tdimension, Tdimension, TElement> rotation;
  for (size_t row = 0; row < Tdimension; ++row)
  {
    for (size_t column = 0; column < Tdimension; ++column)
    {
      rotation[row][column] = transformation[row][column];
    }
  }
  assert(math::IsEqual(rotation.Determinant(), 1));
#endif

  for (size_t i = 0; i < this->NumberOfControlPoints(); ++i)
  {
    this->control_points[i] = transformation.MultiplyHomogeneously(this->control_points[i]);
  }
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tBezierCurve UpdateBoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tBezierCurve<Tdimension, TElement, Tdegree>::UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const
{
  for (size_t i = 0; i < this->NumberOfControlPoints(); ++i)
  {
    bounding_box.Add(this->control_points[i]);
  }
}

//----------------------------------------------------------------------
// tBezierCurve UpdateCenterOfGravity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tBezierCurve<Tdimension, TElement, Tdegree>::UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const
{
  for (size_t i = 0; i < this->NumberOfControlPoints(); ++i)
  {
    center_of_gravity += this->control_points[i];
  }
  center_of_gravity /= this->NumberOfControlPoints();
}

//----------------------------------------------------------------------
// Operators for rrlib_canvas
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_

template <typename TElement, unsigned int Tdegree>
canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tBezierCurve<2, TElement, Tdegree> &bezier_curve)
{
  canvas.DrawBezierCurve(bezier_curve.ControlPoints(), bezier_curve.ControlPoints() + bezier_curve.NumberOfControlPoints());

  return canvas;
}

#endif

//----------------------------------------------------------------------
// Operators for rrlib_serialization
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tBezierCurve<Tdimension, TElement, Tdegree> &curve)
{
  for (size_t i = 0; i < curve.NumberOfControlPoints(); ++i)
  {
    stream << curve.ControlPoints()[i];
  }
  return stream;
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tBezierCurve<Tdimension, TElement, Tdegree> &curve)
{
  for (size_t i = 0; i < curve.NumberOfControlPoints(); ++i)
  {
    typename tBezierCurve<Tdimension, TElement, Tdegree>::tPoint control_point;
    stream >> control_point;
    curve.SetControlPoint(i, control_point);
  }
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
