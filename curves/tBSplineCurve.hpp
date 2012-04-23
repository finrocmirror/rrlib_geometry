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
/*!\file    tBSplineCurve.cpp
 *
 * \author  Patrick Fleischmann
 *
 * \date    Apr 18, 2012
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
// tBSplineCurve constructor
//----------------------------------------------------------------------
template<size_t Tdimension, typename TElement, unsigned int Tdegree>
template<typename TIterator, typename TKnotIterator>
tBSplineCurve<Tdimension, TElement, Tdegree>::tBSplineCurve(TIterator control_points_begin, TIterator control_points_end, TKnotIterator knots_begin, TKnotIterator knots_end) :
  tSplineCurve(control_points_begin, control_points_end)
{
  // copy knots
  std::copy(knots_begin, knots_end, std::back_inserter(this->knots));
  assert(this->knots.size() == this->ControlPoints().size() + Tdegree + 1);
  this->CalculateBezierControlPoints();
}

template<size_t Tdimension, typename TElement, unsigned int Tdegree>
template<typename TIterator>
tBSplineCurve<Tdimension, TElement, Tdegree>::tBSplineCurve(TIterator begin, TIterator end) :
  tSplineCurve(begin, end)
{
  this->CalculateKnotVector();
  this->CalculateBezierControlPoints();
}

//template<size_t Tdimension, typename TElement, unsigned int Tdegree>
//void tBSplineCurve<Tdimension, TElement, Tdegree>::AppendControlPoint(const typename tShape::tPoint &point)
//{
//  tSplineCurve::AppendControlPoint(point);
//  this->CalculateKnotVector();
//  this->CalculateBezierControlPoints();
//}
//
//----------------------------------------------------------------------
// tBSplineCurve SetChanged
//----------------------------------------------------------------------
template<size_t Tdimension, typename TElement, unsigned int Tdegree>
void tBSplineCurve<Tdimension, TElement, Tdegree>::SetChanged()
{
  tSplineCurve::SetChanged();
  this->CalculateKnotVector();
  this->CalculateBezierControlPoints();
}

template<size_t Tdimension, typename TElement, unsigned int Tdegree>
void tBSplineCurve<Tdimension, TElement, Tdegree>::CalculateKnotVector()
{
  this->knots.clear();
  assert(this->NumberOfControlPoints() > Tdegree);
  // calculate knot vector
  unsigned int length = this->NumberOfControlPoints() + Tdegree + 1;
  this->knots.reserve(length);

  for (unsigned int i = 0; i < length; ++i)
  {
    if (i < Tdegree + 1)
    {
      this->knots.push_back(0);
    }
    else if ((Tdegree + 1) <= i && i <= this->NumberOfControlPoints())
    {
      // inner knot vector (uniform)
      this->knots.push_back(1.0 / (this->NumberOfControlPoints() - Tdegree) * (i - Tdegree));
    }
    else if (i > this->NumberOfControlPoints())
    {
      this->knots.push_back(1.0);
    }
  }
}

template<size_t Tdimension, typename TElement, unsigned int Tdegree>
void tBSplineCurve<Tdimension, TElement, Tdegree>::CalculateBezierControlPoints()
{
  this->bezier_control_points.clear();
  std::vector<double> new_knots;
  new_knots.reserve(this->knots.size() * Tdegree);
  std::copy(this->knots.begin(), this->knots.end(), std::back_inserter(new_knots));
  std::copy(this->ControlPoints().begin(), this->ControlPoints().end(), std::back_inserter(this->bezier_control_points));

  double knot = new_knots[0];
  unsigned int multiplicity = 1;

  for (std::vector<double>::iterator it = (++new_knots.begin()); it < new_knots.end(); ++it)
  {
    if (knot == *it)
    {
      multiplicity++;
    }
    else
    {
      if (multiplicity < Tdegree)
      {
        for (unsigned int s = multiplicity; s < Tdegree; s++)
        {
          this->bezier_control_points = InsertKnot((it - new_knots.begin()) - multiplicity, new_knots, knot, this->bezier_control_points);
          new_knots.insert(it, knot);
        }
        it += Tdegree - multiplicity;
      }
      if (it < new_knots.end() - 1)
      {
        knot = *it;
        multiplicity = 1;
      }
    }
  }
}

template<size_t Tdimension, typename TElement, unsigned int Tdegree>
std::vector<typename tBSplineCurve<Tdimension, TElement, Tdegree>::tShape::tPoint> tBSplineCurve<Tdimension, TElement, Tdegree>::InsertKnot(int at, const std::vector<double> &knots_before_insertion, double knot, const std::vector <
    typename tShape::tPoint > &control_points) const
{
  std::vector<typename tShape::tPoint> new_control_points;
  new_control_points.reserve(control_points.size() + 1);

  // copy unaffected points (index < at-Tdegree+1)
  std::copy(control_points.begin(), control_points.begin() + at - Tdegree + 1, std::back_inserter(new_control_points));

  // recalculate control points affected by knot insertion
  for (int i = at - Tdegree + 1; i <= at; i++)
  {
    double a = (knot - knots_before_insertion[i]) / (knots_before_insertion[i + Tdegree] - knots_before_insertion[i]);
    new_control_points.push_back((1 - a) * control_points[i - 1] + a * control_points[i]);
  }

  // copy unaffected points (index > at)
  std::copy(control_points.begin() + at, control_points.end(), std::back_inserter(new_control_points));

  return new_control_points;
}

//----------------------------------------------------------------------
// tBSplineCurve GetSegmentForParameter
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
unsigned int tBSplineCurve<Tdimension, TElement, Tdegree>::GetSegmentForParameter(typename tSplineCurve::tParameter t)
{
  assert((this->knots.front() <= t) && (t <= this->knots.back()));
  auto it = std::lower_bound(this->knots.begin(), this->knots.end(), t);

  return static_cast<unsigned int>(t == this->knots.front() ? 0 : std::distance(this->knots.begin(), it) - 1);
}

//----------------------------------------------------------------------
// tBSplineCurve GetLocalParameter
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
typename tSplineCurve<Tdimension, TElement, Tdegree>::tParameter tBSplineCurve<Tdimension, TElement, Tdegree>::GetLocalParameter(typename tSplineCurve::tParameter t)
{
  unsigned int start = this->GetSegmentForParameter(t);
  unsigned int stop = start + 1;
  while (this->knots[start] == this->knots[stop])
  {
    stop++;
    assert(stop < this->knots.size());
  }
  return (t - this->knots[start]) / (this->knots[stop] - this->knots[start]);
}

//----------------------------------------------------------------------
// tBSplineCurve CreateBezierCurveForSegment
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, unsigned int Tdegree>
std::shared_ptr<const typename tSplineCurve<Tdimension, TElement, Tdegree>::tBezierCurve> tBSplineCurve<Tdimension, TElement, Tdegree>::CreateBezierCurveForSegment(unsigned int i) const
{
  std::vector<typename tShape::tPoint> segment_control_points;
  std::copy(this->bezier_control_points.begin() + i * Tdegree, this->bezier_control_points.begin() + i * Tdegree + Tdegree + 1, std::back_inserter(segment_control_points));
  return std::shared_ptr<const typename tSplineCurve::tBezierCurve>(new typename tSplineCurve::tBezierCurve(segment_control_points.begin(), segment_control_points.end()));
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
