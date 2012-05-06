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
/*!\file    test_geometries.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-26
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>
#include <iostream>
#include <vector>

#include "rrlib/highgui_wrapper/tWindow.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tShape.h"
#include "rrlib/geometry/tLine.h"
#include "rrlib/geometry/curves/tBezierCurve.h"
#include "rrlib/geometry/curves/tUniformBSplineCurve.h"
#include "rrlib/geometry/curves/tCardinalSplineCurve.h"
#include "rrlib/geometry/functions.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::highgui;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef double tElement;
typedef rrlib::math::tVector<2, tElement> tVector;
typedef rrlib::geometry::tShape<2, tElement> tShape;
typedef rrlib::geometry::tPoint<2, tElement> tPoint;
typedef rrlib::geometry::tLine<2, tElement> tLine;
typedef rrlib::geometry::tLineSegment<2, tElement> tLineSegment;
typedef rrlib::geometry::tBezierCurve<2, tElement, 3> tBezierCurve;
typedef rrlib::geometry::tSplineCurve<2, tElement, 3> tSplineCurve;
typedef rrlib::geometry::tUniformBSplineCurve<2, tElement> tConcreteSplineCurve;
//typedef rrlib::geometry::tCardinalSplineCurve<2, tElement> tConcreteSplineCurve;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

void DrawBoundingBox(tWindow &window, const tShape &shape)
{
  window.DrawRectangleNormalized(shape.BoundingBox().Min().X(), shape.BoundingBox().Min().Y(),
                                 shape.BoundingBox().Max().X(), shape.BoundingBox().Max().Y());
}

void DrawPoint(tWindow &window, const tPoint &point)
{
  window.DrawCircleNormalized(point.X(), point.Y(), 0.01, true);
}

void DrawLineSegment(tWindow &window, const tLineSegment &line_segment)
{
  window.DrawLineNormalized(line_segment.Begin().X(), line_segment.Begin().Y(), line_segment.End().X(), line_segment.End().Y());
}

void DrawLine(tWindow &window, const tLine &line)
{
  tPoint end_points[4];
  size_t index = 0;
  index += line.GetIntersection(end_points[index], tLineSegment(tPoint(0, 0), tPoint(0, 1))) ? 1 : 0;
  index += line.GetIntersection(end_points[index], tLineSegment(tPoint(0, 1), tPoint(1, 1))) ? 1 : 0;
  index += line.GetIntersection(end_points[index], tLineSegment(tPoint(1, 1), tPoint(1, 0))) ? 1 : 0;
  index += line.GetIntersection(end_points[index], tLineSegment(tPoint(1, 0), tPoint(0, 0))) ? 1 : 0;
  assert(index == 2);
  DrawLineSegment(window, tLineSegment(end_points[0], end_points[1]));
}

void DrawControlPoints(tWindow &window, const std::vector<tPoint> &data)
{
  for (std::vector<tPoint>::const_iterator it = data.begin(); it != data.end(); it++)
  {
    DrawPoint(window, *it);
  }
}

void DrawControlPolygon(tWindow &window, const tBezierCurve &bezier_curve)
{
  for (size_t i = 1; i < bezier_curve.NumberOfControlPoints(); ++i)
  {
    const tPoint &start(bezier_curve.ControlPoints()[i - 1]);
    const tPoint &stop(bezier_curve.ControlPoints()[i]);

    window.DrawLineNormalized(start.X(), start.Y(), stop.X(), stop.Y());
  }
}

void DrawControlPolygon(tWindow &window, const tSplineCurve &spline_curve)
{
  for (size_t i = 1; i < spline_curve.NumberOfControlPoints(); ++i)
  {
    const tPoint &start(spline_curve.ControlPoints()[i - 1]);
    const tPoint &stop(spline_curve.ControlPoints()[i]);

    window.DrawLineNormalized(start.X(), start.Y(), stop.X(), stop.Y());
  }
}

void DrawBezierCurve(tWindow &window, const tBezierCurve &bezier_curve, float epsilon = 1.0E-6)
{
  if (bezier_curve.GetTwist() < epsilon)
  {
    DrawControlPolygon(window, bezier_curve);
    return;
  }

  std::pair<tBezierCurve, tBezierCurve> subdivision(bezier_curve.GetSubdivision());
  DrawBezierCurve(window, subdivision.first, epsilon);
  DrawBezierCurve(window, subdivision.second, epsilon);
}

void DrawSplineCurve(tWindow &window, const tSplineCurve &spline_curve, float epsilon = 1.0E-6)
{
  for (size_t i = 0; i < spline_curve.NumberOfSegments(); ++i)
  {
    DrawBezierCurve(window, *spline_curve.GetBezierCurveForSegment(i), epsilon);
  }
}

int main(int argc, char **argv)
{
  tWindow &window(tWindow::GetInstance("Test Geometries", 500, 500));

  std::cout << std::endl << "=== A line segment with its bounding box ===" << std::endl;

  tLineSegment line_segment(tPoint(0.1, 0.2), tPoint(0.9, 0.4));

  window.Clear();
  window.SetColor(0);
  DrawLineSegment(window, line_segment);
  window.SetColor(1);
  DrawBoundingBox(window, line_segment);
  window.Render();

  std::cout << std::endl << "=== A line ===" << std::endl;

  tLine line(tPoint(0.1, 0.2), tPoint(0.9, 0.4));

  window.Clear();
  window.SetColor(1);
  DrawLine(window, line);
  window.Render();

  std::cout << std::endl << "=== Control points ===" << std::endl;

  std::vector<tPoint> control_points;
  control_points.push_back(tPoint(0.1, 0.2));
  control_points.push_back(tPoint(0.3, 0.5));
  control_points.push_back(tPoint(0.7, 0.6));
  control_points.push_back(tPoint(0.5, 0.2));

  window.Clear();
  window.SetColor(0);
  DrawControlPoints(window, control_points);
  window.Render();

  std::cout << std::endl << "=== Control polygon and bounding box ===" << std::endl;

  tBezierCurve bezier_curve(control_points.begin(), control_points.end());

  window.Clear();
  window.SetColor(1);
  DrawControlPolygon(window, bezier_curve);
  window.SetColor(2);
  DrawBoundingBox(window, bezier_curve);
  window.Render();

  std::cout << std::endl << "=== Bezier curve ===" << std::endl;

  window.SetColor(3);
  DrawBezierCurve(window, bezier_curve);
  window.Render();


  for (int i = 0; i <= 100; ++i)
  {
    double t = i / 100.0;
    tPoint p = bezier_curve(t);
    window.Clear();
    DrawBezierCurve(window, bezier_curve);
    DrawPoint(window, p);

    double curvature = rrlib::geometry::GetCurvature(bezier_curve, t);
    if (curvature > 0.0)
    {
      double radius = 1.0 / curvature;

      tBezierCurve::tDerivative first(bezier_curve.GetDerivative());
      tPoint m(p + (first(t).Normalized().Rotated(-M_PI_2) * radius));

      window.SetColor(1);
      window.DrawCircleNormalized(m.X(), m.Y(), radius, false);
    }

    window.Render();
  }

  std::cout << std::endl << "=== Bezier curve intersects line ===" << std::endl;

  window.Clear();
  window.SetColor(0);
  line.Translate(tVector(0, 0.1));
  DrawLine(window, line);
  window.SetColor(1);
  DrawBezierCurve(window, bezier_curve);

  window.Render();

  std::vector<tPoint> intersection_points;
  std::vector<tElement> intersection_parameters;
  bezier_curve.GetIntersections(intersection_points, intersection_parameters, line);
  std::cout << "number of intersections: " << intersection_points.size() << std::endl;

  window.SetColor(2);
  for (std::vector<tPoint>::iterator it = intersection_points.begin(); it != intersection_points.end(); ++it)
  {
    std::cout << *it << std::endl;
    DrawPoint(window, *it);
  }
  window.Render();

  std::cout << std::endl << "=== Bezier curve intersects curve ===" << std::endl;

  tBezierCurve bezier_curve2(tPoint(0.2, 0.1), tPoint(0.5, 0.3), tPoint(0.6, 0.7), tPoint(0.4, 0.9));

  rrlib::math::tAngleDeg angle = -40;
  tPoint position = bezier_curve2.CenterOfGravity();
  bezier_curve2.Translate(-position);
  bezier_curve2.Rotate(rrlib::math::Get2DRotationMatrix<double>(angle));
  bezier_curve2.Translate(position);
  bezier_curve2.Translate(tPoint(0, -0.1));

  window.Clear();
  window.SetColor(0);
  DrawBezierCurve(window, bezier_curve2);
  window.SetColor(1);
  DrawBezierCurve(window, bezier_curve);

  window.Render();

  intersection_points.clear();
  intersection_parameters.clear();

  bezier_curve.GetIntersections(intersection_points, intersection_parameters, bezier_curve2);
  std::cout << "number of intersections: " << intersection_points.size() << std::endl;

  window.SetColor(2);
  for (std::vector<tPoint>::iterator it = intersection_points.begin(); it != intersection_points.end(); ++it)
  {
    DrawPoint(window, *it);
  }
  window.Render();

  std::cout << std::endl << "=== Spline with bounding box ===" << std::endl;

  control_points.clear();
  control_points.push_back(tPoint(0.1, 0.2));
  control_points.push_back(tPoint(0.1, 0.2));
  control_points.push_back(tPoint(0.1, 0.2));
  control_points.push_back(tPoint(0.3, 0.5));
  control_points.push_back(tPoint(0.7, 0.6));
  control_points.push_back(tPoint(0.9, 0.4));

  tConcreteSplineCurve spline(control_points.begin(), control_points.end());

  window.Clear();
  window.SetColor(1);
  DrawControlPolygon(window, spline);
  window.SetColor(2);
  DrawBoundingBox(window, spline);
  window.SetColor(3);
  DrawSplineCurve(window, spline);
  window.Render();

  std::cout << std::endl << "=== Adding point ===" << std::endl;

  spline.AppendControlPoint(tPoint(0.75, 0.7));
  window.Clear();
  window.SetColor(1);
  DrawControlPolygon(window, spline);
  window.SetColor(2);
  DrawBoundingBox(window, spline);
  window.SetColor(3);
  DrawSplineCurve(window, spline);
  window.Render();

  std::cout << std::endl << "=== Adding point ===" << std::endl;

  spline.AppendControlPoint(tPoint(0.5, 0.8));
  window.Clear();
  window.SetColor(1);
  DrawControlPolygon(window, spline);
  window.SetColor(2);
  DrawBoundingBox(window, spline);
  window.SetColor(3);
  DrawSplineCurve(window, spline);
  window.Render();

  std::cout << std::endl << "=== Adding point ===" << std::endl;

  spline.AppendControlPoint(tPoint(0.2, 0.4));
  window.Clear();
  window.SetColor(1);
  DrawControlPolygon(window, spline);
  window.SetColor(2);
  DrawBoundingBox(window, spline);
  window.SetColor(3);
  DrawSplineCurve(window, spline);
  window.Render();

  std::cout << std::endl << "=== Adding point ===" << std::endl;

  spline.AppendControlPoint(tPoint(0.2, 0.4));
  window.Clear();
  window.SetColor(1);
  DrawControlPolygon(window, spline);
  window.SetColor(2);
  DrawBoundingBox(window, spline);
  window.SetColor(3);
  DrawSplineCurve(window, spline);
  window.Render();

  std::cout << std::endl << "=== Adding point ===" << std::endl;

  spline.AppendControlPoint(tPoint(0.2, 0.4));
  window.Clear();
  window.SetColor(1);
  DrawControlPolygon(window, spline);
  window.SetColor(2);
  DrawBoundingBox(window, spline);
  window.SetColor(3);
  DrawSplineCurve(window, spline);
  window.Render();

  std::cout << std::endl << "=== Spline intersects line ===" << std::endl;

  window.Clear();
  window.SetColor(0);
  line.Translate(tVector(0, 0.1));
  DrawLine(window, line);
  window.SetColor(1);
  DrawSplineCurve(window, spline);

  window.Render();

  intersection_points.clear();
  intersection_parameters.clear();
  spline.GetIntersections(intersection_points, intersection_parameters, line);
  std::cout << "number of intersections: " << intersection_points.size() << std::endl;

  window.SetColor(2);
  for (std::vector<tPoint>::iterator it = intersection_points.begin(); it != intersection_points.end(); ++it)
  {
    std::cout << *it << std::endl;
    DrawPoint(window, *it);
  }
  window.Render();

  std::cout << std::endl << "=== Closest point to Bezier curve ===" << std::endl;

  window.Clear();
  window.SetColor(0);
  DrawBezierCurve(window, bezier_curve);
  window.Render();

  for (int i = 0; i < 10; ++i)
  {
    tPoint reference_point(drand48(), drand48());
    tPoint closest_point(bezier_curve.GetClosestPoint(reference_point));

    window.SetColor(3);
    DrawLineSegment(window, tLineSegment(reference_point, closest_point));
    window.SetColor(1);
    DrawPoint(window, reference_point);
    window.SetColor(2);
    DrawPoint(window, closest_point);
    window.Render();
  }

  std::cout << std::endl << "=== Closest point to spline curve ===" << std::endl;

  window.Clear();
  window.SetColor(0);
  DrawSplineCurve(window, spline);
  window.Render();

  for (int i = 0; i < 10; ++i)
  {
    tPoint reference_point(drand48(), drand48());
    tPoint closest_point(spline.GetClosestPoint(reference_point));

    window.SetColor(3);
    DrawLineSegment(window, tLineSegment(reference_point, closest_point));
    window.SetColor(1);
    DrawPoint(window, reference_point);
    window.SetColor(2);
    DrawPoint(window, closest_point);
    window.Render();
  }

  window.Render();

  tWindow::ReleaseAllInstances();

  return EXIT_SUCCESS;
}
