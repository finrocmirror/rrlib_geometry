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
/*!\file    tLineSegment.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-27
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tLUDecomposition.h"

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
// tLineSegment constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement>::tLineSegment()
{
  this->end[0] = 1;
}

template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement>::tLineSegment(const typename tShape::tPoint &begin, const typename tShape::tPoint &end)
  : tLine(begin, end - begin),
    end(end)
{}

//----------------------------------------------------------------------
// tLineSegment Set
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLineSegment<Tdimension, TElement>::Set(const typename tShape::tPoint &begin, const typename tShape::tPoint &end)
{
  tLine::Set(begin, end - begin);
  this->end = end;
}

//----------------------------------------------------------------------
// tLineSegment GetClosestPoint
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tShape<Tdimension, TElement>::tPoint tLineSegment<Tdimension, TElement>::GetClosestPoint(const typename tShape::tPoint &reference_point) const
{
  math::tVector<Tdimension, TElement> from_begin_to_point(reference_point - this->Begin());
  math::tVector<Tdimension, TElement> from_end_to_point(reference_point - this->End());

  // angle between direction and vector begin->point larger than 90°
  if (from_begin_to_point * this->Direction() < 0)
  {
    return this->Begin();
  }

  // angle between direction and vector end->point larger than 90°
  if (from_end_to_point * this->Direction() > 0)
  {
    return this->End();
  }

  return tLine::GetClosestPoint(reference_point);
}

//----------------------------------------------------------------------
// tLineSegment GetIntersection
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
std::pair<bool, tLineSegment<Tdimension, TElement>> tLineSegment<Tdimension, TElement>::GetIntersection(typename tShape::tBoundingBox &bounding_box) const
{
  typedef typename tShape::tPoint tPoint;
  typedef std::pair<double, tPoint> tIntersection;                     // Intersection point (first is relative distance to line_segment.Begin() with range: 0-1)
  std::array<std::pair<double, tPoint>, Tdimension * 2> intersections; // Intersection points (cannot be more than two - corners, however, may be added multiple times)
  size_t intersection_count = 0;                                       // Number of intersections
  typedef std::pair<uint, tLineSegment<Tdimension, TElement>> tResult;
  typedef rrlib::math::tVector<Tdimension, TElement> tVector;

  // Are begin/end points already in bounding box?
  if (bounding_box.Contains(Begin()))
  {
    intersections[intersection_count] = tIntersection(0, Begin());
    intersection_count++;
  }
  if (bounding_box.Contains(End()))
  {
    intersections[intersection_count] = tIntersection(1, End());
    intersection_count++;
  }
  if (intersection_count == 2)
  {
    return tResult(true, *this);
  }

  // Check whether/where line segments cross sides of bounding box
  tVector diff_vector = End() - Begin();

  // To avoid numeric instabilities/asymmetries, bounding box is centered in zero
  tVector box_dimension = bounding_box.Max() - bounding_box.Min();
  tVector box_center = (bounding_box.Max() + bounding_box.Min()) * 0.5;
  tBoundingBox<Tdimension, TElement> centered_box;
  centered_box.Add(tPoint(box_dimension * 0.5));
  centered_box.Add(tPoint(-box_dimension * 0.5));
  tPoint begin_relative_to_box(Begin() - box_center);
  tPoint end_relative_to_box(End() - box_center);
  tPoint interpolated;
  for (size_t i = 0; i < Tdimension; i++)
  {
    // min side of dimension
    if ((begin_relative_to_box[i] < centered_box.Min()[i] && end_relative_to_box[i] >= centered_box.Min()[i]) ||
        (end_relative_to_box[i] < centered_box.Min()[i] && begin_relative_to_box[i] >= centered_box.Min()[i]))
    {
      double distance_begin = std::fabs(centered_box.Min()[i] - begin_relative_to_box[i]);
      double distance_end = std::fabs(centered_box.Min()[i] - end_relative_to_box[i]);
      double relative_begin_distance = distance_begin / (distance_begin + distance_end);
      if (distance_begin <= distance_end)
      {
        interpolated = begin_relative_to_box + (diff_vector * relative_begin_distance);
      }
      else
      {
        double relative_end_distance = distance_end / (distance_begin + distance_end);
        interpolated = end_relative_to_box - (diff_vector * relative_end_distance);
      }
      interpolated[i] = centered_box.Min()[i]; // eliminate any numeric errors in this dimension at least
      if (centered_box.Contains(interpolated))
      {
        tPoint p = interpolated + box_center;
        p[i] = bounding_box.Min()[i];
        intersections[intersection_count] = tIntersection(relative_begin_distance, p);
        intersection_count++;
      }
    }
    // max side of dimension
    if ((begin_relative_to_box[i] > centered_box.Max()[i] && end_relative_to_box[i] <= centered_box.Max()[i]) ||
        (end_relative_to_box[i] > centered_box.Max()[i] && begin_relative_to_box[i] <= centered_box.Max()[i]))
    {
      double distance_begin = std::fabs(centered_box.Max()[i] - begin_relative_to_box[i]);
      double distance_end = std::fabs(centered_box.Max()[i] - end_relative_to_box[i]);
      double relative_begin_distance = distance_begin / (distance_begin + distance_end);
      if (distance_begin <= distance_end)
      {
        interpolated = begin_relative_to_box + (diff_vector * relative_begin_distance);
      }
      else
      {
        double relative_end_distance = distance_end / (distance_begin + distance_end);
        interpolated = end_relative_to_box - (diff_vector * relative_end_distance);
      }
      interpolated[i] = centered_box.Max()[i]; // eliminate any numeric errors in this dimension at least
      if (centered_box.Contains(interpolated))
      {
        tPoint p = interpolated + box_center;
        p[i] = bounding_box.Max()[i];
        intersections[intersection_count] = tIntersection(relative_begin_distance, p);
        intersection_count++;
      }
    }
  }

  if (!intersection_count)
  {
    return tResult(false, tLineSegment<Tdimension, TElement>());
  }

  // Prepare result
  tIntersection intersection_begin = intersections[0];
  tIntersection intersection_end = intersections[0];
  for (size_t i = 1; i < intersection_count; i++)
  {
    const tIntersection& intersection = intersections[i];
    if (intersection.first < intersection_begin.first)
    {
      intersection_begin = intersection;
    }
    if (intersection.first > intersection_end.first)
    {
      intersection_end = intersection;
    }
  }

  return tResult(true, tLineSegment<Tdimension, TElement>(intersection_begin.second, intersection_end.second));
}

//----------------------------------------------------------------------
// tLineSegment Translate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement> &tLineSegment<Tdimension, TElement>::Translate(const math::tVector<Tdimension, TElement> &translation)
{
  tLine::Translate(translation);
  this->end += translation;
  return *this;
}

//----------------------------------------------------------------------
// tLineSegment Rotate
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement> &tLineSegment<Tdimension, TElement>::Rotate(const math::tMatrix<Tdimension, Tdimension, TElement> &rotation)
{
  tLine::Rotate(rotation);
  this->end = rotation * this->end;
  return *this;
}

//----------------------------------------------------------------------
// tLineSegment Transform
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tLineSegment<Tdimension, TElement> &tLineSegment<Tdimension, TElement>::Transform(const math::tMatrix < Tdimension + 1, Tdimension + 1, TElement > &transformation)
{
  tLine::Transform(transformation);
  this->end = transformation.MultiplyHomogeneously(this->end);
  return *this;
}

//----------------------------------------------------------------------
// tLineSegment UpdateBoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLineSegment<Tdimension, TElement>::UpdateBoundingBox(typename tShape::tBoundingBox &bounding_box) const
{
  bounding_box.Add(this->Begin());
  bounding_box.Add(this->End());
}

//----------------------------------------------------------------------
// tLineSegment UpdateCenterOfGravity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tLineSegment<Tdimension, TElement>::UpdateCenterOfGravity(typename tShape::tPoint &center_of_gravity) const
{
  center_of_gravity = 0.5 * (this->Begin() + this->End());
}

//----------------------------------------------------------------------
// Operators for rrlib_canvas
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_CANVAS_PRESENT_

template <typename TElement>
canvas::tCanvas2D &operator << (canvas::tCanvas2D &canvas, const tLineSegment<2, TElement> &line_segment)
{
  canvas.DrawLineSegment(line_segment.Begin(), line_segment.End());

  return canvas;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
