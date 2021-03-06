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
/*!\file    tSplineCurve.hpp
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
#include <algorithm>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace declarations
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
// tSplineCurve constructor
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
tSplineCurve<Tdimension, TElement, Tdegree>::tSplineCurve() :
  control_points(Tdegree + 1, tShape::tPoint::Zero())
{
  static_assert(Tdegree > 0, "The degree of spline curves must be greater than zero");
  assert(control_points.size() > Tdegree && "A spline curve needs at least degree + 1 control points");
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <typename TIterator>
tSplineCurve<Tdimension, TElement, Tdegree>::tSplineCurve(TIterator begin, TIterator end)
{
  static_assert(Tdegree > 0, "The degree of spline curves must be greater than zero");
  std::copy(begin, end, std::back_inserter(this->control_points));
  assert(control_points.size() > Tdegree && "A spline curve needs at least degree + 1 control points");
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <typename ... TPoints>
tSplineCurve<Tdimension, TElement, Tdegree>::tSplineCurve(const typename tShape::tPoint &p1, const typename tShape::tPoint &p2, const TPoints &... rest)
{
  static_assert(Tdegree > 0, "The degree of spline curves must be greater than zero");
  static_assert(sizeof...(rest) + 2 > Tdegree, "A spline curve needs at least degree + 1 control points");
  util::ProcessVariadicValues([this](const typename tShape::tPoint & x)
  {
    this->control_points.push_back(x);
  },
  p1, p2, rest...);
}

//----------------------------------------------------------------------
// tSplineCurve NumberOfSegments
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const unsigned int tSplineCurve<Tdimension, TElement, Tdegree>::NumberOfSegments() const
{
  return this->control_points.size() - Tdegree;
};

//----------------------------------------------------------------------
// tSplineCurve SetControlPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::SetControlPoint(size_t i, const typename tShape::tPoint &point)
{
  assert(i < this->control_points.size());
  this->control_points[i] = point;
  this->SetChanged();
};

//----------------------------------------------------------------------
// tSplineCurve AppendControlPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::AppendControlPoint(const typename tShape::tPoint &point)
{
  this->control_points.push_back(point);
  this->SetChanged();
};

//----------------------------------------------------------------------
// tSplineCurve InsertControlPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::InsertControlPoint(size_t position, const typename tShape::tPoint &point)
{
  assert(position < this->control_points.size());
  this->control_points.insert(this->control_points.begin() + position, point);
  this->SetChanged();
};

//----------------------------------------------------------------------
// tSplineCurve SetControlPoints
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <typename TIterator>
void tSplineCurve<Tdimension, TElement, Tdegree>::SetControlPoints(TIterator begin, TIterator end)
{
  this->control_points.assign(begin, end);
  assert(this->control_points.size() > Tdegree && "A spline curve needs at least degree + 1 control points");
  this->SetChanged();
}

//----------------------------------------------------------------------
// tSplineCurve Evaluation: operator ()
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tShape<Tdimension, TElement>::tPoint tSplineCurve<Tdimension, TElement, Tdegree>::operator()(tParameter t) const
{
  assert((0 <= t) && (t <= this->NumberOfSegments()));
  std::shared_ptr<const tBezierCurve> bezier_curve = this->GetBezierCurveForParameter(t, t);
  return bezier_curve->operator()(t);
}

//----------------------------------------------------------------------
// tSplineCurve GetBezierCurveForParameter
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
std::shared_ptr<const typename tSplineCurve<Tdimension, TElement, Tdegree>::tBezierCurve> tSplineCurve<Tdimension, TElement, Tdegree>::GetBezierCurveForParameter(tParameter t) const
{
  return this->GetBezierCurveForSegment(this->GetSegmentForParameter(t));
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
std::shared_ptr<const typename tSplineCurve<Tdimension, TElement, Tdegree>::tBezierCurve> tSplineCurve<Tdimension, TElement, Tdegree>::GetBezierCurveForParameter(tParameter t, tParameter &local_t) const
{
  local_t = this->GetLocalParameter(t);
  return this->GetBezierCurveForParameter(t);
}

//----------------------------------------------------------------------
// tSplineCurve GetBezierCurveForSegment
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
std::shared_ptr<const typename tSplineCurve<Tdimension, TElement, Tdegree>::tBezierCurve> tSplineCurve<Tdimension, TElement, Tdegree>::GetBezierCurveForSegment(unsigned int i) const
{
  assert(i < this->NumberOfSegments());
  this->bezier_curve_cache.resize(this->NumberOfSegments());
  if (!this->bezier_curve_cache[i])
  {
    this->bezier_curve_cache[i] = this->CreateBezierCurveForSegment(i);
  }
  return this->bezier_curve_cache[i];
}

//----------------------------------------------------------------------
// tSplineCurve GetIntersections
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <unsigned int Tother_degree>
void tSplineCurve<Tdimension, TElement, Tdegree>::GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
    const tSplineCurve<Tdimension, TElement, Tother_degree> &other_spline) const
{
  for (unsigned int i = 0; i < this->NumberOfSegments(); ++i)
  {
    size_t last_size = intersection_parameters.size();
    std::shared_ptr<const tBezierCurve> bezier_curve = this->GetBezierCurveForSegment(i);
    for (unsigned int k = 0; k < other_spline.NumberOfSegments(); ++k)
    {
      bezier_curve->GetIntersections(intersection_points, intersection_parameters, *other_spline.GetBezierCurveForSegment(k));
    }
    for (size_t k = last_size; k < intersection_parameters.size(); ++k)
    {
      intersection_parameters[k] += i * 1.0;
    }
  }
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <unsigned int Tother_degree>
void tSplineCurve<Tdimension, TElement, Tdegree>::GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
    const geometry::tBezierCurve<Tdimension, TElement, Tother_degree> &bezier_curve) const
{
  for (unsigned int i = 0; i < this->NumberOfSegments(); ++i)
  {
    size_t last_size = intersection_parameters.size();
    this->GetBezierCurveForSegment(i).GetIntersections(intersection_points, intersection_parameters, bezier_curve);
    for (size_t k = last_size; k < intersection_parameters.size(); ++k)
    {
      intersection_parameters[k] += i * 1.0;
    }
    if (last_size > 0 && intersection_parameters.size() > last_size)
    {
      // at least one intersection existed before this test and at least one was added
      if (intersection_parameters[last_size] == intersection_parameters[last_size - 1])
      {
        intersection_points.erase(intersection_points.begin() + last_size);
        intersection_parameters.erase(intersection_parameters.begin() + last_size);
      }
    }
  }
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
    const tLine<Tdimension, TElement> &line) const
{
  for (unsigned int i = 0; i < this->NumberOfSegments(); ++i)
  {
    size_t last_size = intersection_parameters.size();
    std::shared_ptr<const tBezierCurve> bezier_curve = this->GetBezierCurveForSegment(i);
    bezier_curve->GetIntersections(intersection_points, intersection_parameters, line);
    for (size_t k = last_size; k < intersection_parameters.size(); ++k)
    {
      intersection_parameters[k] += i * 1.0;
    }
    if (last_size > 0 && intersection_parameters.size() > last_size)
    {
      // at least one intersection existed before this test and at least one was added
      if (intersection_parameters[last_size] == intersection_parameters[last_size - 1])
      {
        intersection_points.erase(intersection_points.begin() + last_size);
        intersection_parameters.erase(intersection_parameters.begin() + last_size);
      }
    }
  }
}

//----------------------------------------------------------------------
// tSplineCurve GetClosestPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tShape<Tdimension, TElement>::tPoint tSplineCurve<Tdimension, TElement, Tdegree>::GetClosestPoint(const typename tShape::tPoint &reference_point) const
{
  typename tShape::tPoint closest_point = this->GetBezierCurveForSegment(0)->GetClosestPoint(reference_point);
  double min_distance = (closest_point - reference_point).Length();

  for (size_t i = 1; i < this->NumberOfSegments(); ++i)
  {
    typename tShape::tPoint candidate = this->GetBezierCurveForSegment(i)->GetClosestPoint(reference_point);
    double distance = (candidate - reference_point).Length();

    if (distance < min_distance)
    {
      min_distance = distance;
      closest_point = candidate;
    }
  }

  return closest_point;
}

//----------------------------------------------------------------------
// tSplineCurve Translate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
tSplineCurve<Tdimension, TElement, Tdegree> &tSplineCurve<Tdimension, TElement, Tdegree>::Translate(const math::tVector<Tdimension, TElement> &translation)
{
  for (typename std::vector<typename tShape::tPoint>::iterator it = this->control_points.begin(); it != this->control_points.end(); ++it)
  {
    *it += translation;
  }
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tSplineCurve Rotate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
tSplineCurve<Tdimension, TElement, Tdegree> &tSplineCurve<Tdimension, TElement, Tdegree>::Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation)
{
  for (typename std::vector<typename tShape::tPoint>::iterator it = this->control_points.begin(); it != this->control_points.end(); ++it)
  {
    *it = rotation **it;
  }
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tSplineCurve Transform
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
tSplineCurve<Tdimension, TElement, Tdegree> &tSplineCurve<Tdimension, TElement, Tdegree>::Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation)
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

  for (typename std::vector<typename tShape::tPoint>::iterator it = this->control_points.begin(); it != this->control_points.end(); ++it)
  {
    *it = transformation.MultiplyHomogeneously(*it);
  }
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tSplineCurve SetChanged
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::SetChanged()
{
  tShape::SetChanged();
  for (auto it = this->bezier_curve_cache.begin(); it != this->bezier_curve_cache.end(); ++it)
  {
    *it = std::shared_ptr<tBezierCurve>();
  }
}

//----------------------------------------------------------------------
// tSplineCurve UpdateBoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const
{
  for (size_t i = 0; i < this->NumberOfSegments(); ++i)
  {
    bounding_box.Add(this->GetBezierCurveForSegment(i)->BoundingBox());
  }
}

//----------------------------------------------------------------------
// tSplineCurve UpdateCenterOfGravity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const
{
  for (size_t i = 0; i < this->NumberOfSegments(); ++i)
  {
    center_of_gravity += this->GetBezierCurveForSegment(i)->CenterOfGravity();
  }
  center_of_gravity /= this->NumberOfSegments();
}

//----------------------------------------------------------------------
// Operators for rrlib_canvas
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_

template <typename TElement, unsigned int Tdegree>
inline canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tSplineCurve<2, TElement, Tdegree> &spline_curve)
{
  unsigned int number_of_segments = spline_curve.NumberOfSegments();
  for (size_t i = 0; i < number_of_segments; ++i)
  {
    canvas << *spline_curve.GetBezierCurveForSegment(i);
  }

  return canvas;
}

#endif

//----------------------------------------------------------------------
// Operators for rrlib_serialization
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tSplineCurve<Tdimension, TElement, Tdegree> &spline)
{
  size_t number_of_control_points = spline.NumberOfControlPoints();
  stream << number_of_control_points;
  for (size_t i = 0; i < number_of_control_points; ++i)
  {
    stream << spline.ControlPoints()[i];
  }
  return stream;
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tSplineCurve<Tdimension, TElement, Tdegree> &spline)
{
  size_t number_of_control_points;
  stream >> number_of_control_points;
  std::vector<tPoint<Tdimension, TElement>> control_points(number_of_control_points);
  for (size_t i = 0; i < number_of_control_points; ++i)
  {
    stream >> control_points[i];
  }
  spline.SetControlPoints(control_points.begin(), control_points.end());
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



