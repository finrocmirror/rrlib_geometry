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
template <typename TIterator>
tSplineCurve<Tdimension, TElement, Tdegree>::tSplineCurve(TIterator begin, TIterator end)
{
  std::copy(begin, end, std::back_inserter(this->control_points));
  assert(control_points.size() > Tdegree);
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <typename TSTLContainer>
tSplineCurve<Tdimension, TElement, Tdegree>::tSplineCurve(const TSTLContainer &control_points)
{
  std::copy(control_points.begin(), control_points.end(), std::back_inserter(this->control_points));
  assert(control_points.size() > Tdegree);
}

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
void tSplineCurve<Tdimension, TElement, Tdegree>::InsertControlPoint(typename std::vector<typename tShape::tPoint>::iterator position, const typename tShape::tPoint &point)
{
  this->control_points.insert(position, point);
  this->SetChanged();
};

//----------------------------------------------------------------------
// tSplineCurve Evaluation: operator ()
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tShape<Tdimension, TElement>::tPoint tSplineCurve<Tdimension, TElement, Tdegree>::operator()(tParameter t) const
{
  assert((0 <= t) && (t <= this->GetNumberOfSegments()));
  const tBezierCurve bezier_curve(this->GetBezierCurveForParameter(t, t));
  return bezier_curve(t);
}

//----------------------------------------------------------------------
// tSplineCurve GetNumberOfSegments
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const unsigned int tSplineCurve<Tdimension, TElement, Tdegree>::GetNumberOfSegments() const
{
  return this->control_points.size() - Tdegree;
};

//----------------------------------------------------------------------
// tSplineCurve GetBezierCurveForParameter
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tSplineCurve<Tdimension, TElement, Tdegree>::tBezierCurve tSplineCurve<Tdimension, TElement, Tdegree>::GetBezierCurveForParameter(tParameter t) const
{
  assert((0 <= t) && (t <= this->GetNumberOfSegments()));
  return this->GetBezierCurveForSegment(static_cast<size_t>(t < this->GetNumberOfSegments() ? t : t - 1.0));
}

template <size_t Tdimension, typename TElement, unsigned int Tdegree>
const typename tSplineCurve<Tdimension, TElement, Tdegree>::tBezierCurve tSplineCurve<Tdimension, TElement, Tdegree>::GetBezierCurveForParameter(tParameter t, tParameter &local_t) const
{
  local_t = t < this->GetNumberOfSegments() ? t - std::trunc(t) : 1.0;
  return this->GetBezierCurveForParameter(t);
}

//----------------------------------------------------------------------
// tSplineCurve GetIntersections
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
template <unsigned int Tother_degree>
void tSplineCurve<Tdimension, TElement, Tdegree>::GetIntersections(std::vector<typename tShape::tPoint> &intersection_points, std::vector<tParameter> &intersection_parameters,
    const tSplineCurve<Tdimension, TElement, Tother_degree> &other_spline) const
{
  for (unsigned int i = 0; i < this->GetNumberOfSegments(); ++i)
  {
    size_t last_size = intersection_parameters.size();
    const tBezierCurve bezier_curve(this->GetBezierCurveForSegment(i));
    for (unsigned int k = 0; k < other_spline.GetNumberOfSegments(); ++k)
    {
      bezier_curve.GetIntersections(intersection_points, intersection_parameters, other_spline.GetBezierCurveForSegment(k));
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
  for (unsigned int i = 0; i < this->GetNumberOfSegments(); ++i)
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
  for (unsigned int i = 0; i < this->GetNumberOfSegments(); ++i)
  {
    size_t last_size = intersection_parameters.size();
    const tBezierCurve bezier_curve(this->GetBezierCurveForSegment(i));
    bezier_curve.GetIntersections(intersection_points, intersection_parameters, line);
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
    *it = rotation * *it;
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
  assert(math::IsEqual(rotation.Determinant(), 0));
#endif

  for (typename std::vector<typename tShape::tPoint>::iterator it = this->control_points.begin(); it != this->control_points.end(); ++it)
  {
    *it = transformation.MultiplyHomogeneously(*it);
  }
  this->SetChanged();
  return *this;
}

//----------------------------------------------------------------------
// tSplineCurve UpdateBoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const
{
  for (size_t i = 0; i < this->GetNumberOfSegments(); ++i)
  {
    bounding_box.Add(this->GetBezierCurveForSegment(i).BoundingBox());
  }
}

//----------------------------------------------------------------------
// tSplineCurve UpdateCenterOfGravity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
void tSplineCurve<Tdimension, TElement, Tdegree>::UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const
{
  for (size_t i = 0; i < this->GetNumberOfSegments(); ++i)
  {
    center_of_gravity += this->GetBezierCurveForSegment(i).CenterOfGravity();
  }
  center_of_gravity /= this->GetNumberOfSegments();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
