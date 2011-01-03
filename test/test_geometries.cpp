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

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tShape.h"
#include "rrlib/geometry/tLine.h"
#include "rrlib/geometry/tBezierCurve.h"
#include "rrlib/highgui_wrapper/tWindow.h"

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
  for (size_t i = 1; i < bezier_curve.cNUMBER_OF_CONTROL_POINTS; ++i)
  {
    const tPoint &start(bezier_curve.GetControlPoint(i - 1));
    const tPoint &stop(bezier_curve.GetControlPoint(i));

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
  control_points.push_back(tPoint(0.9, 0.4));

  window.Clear();
  window.SetColor(0);
  DrawControlPoints(window, control_points);
  window.Render();

  std::cout << std::endl << "=== Control polygon and bounding box ===" << std::endl;

  tBezierCurve bezier_curve(control_points);

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

  std::vector<tPoint> control_points2;
  control_points2.push_back(tPoint(0.2, 0.1));
  control_points2.push_back(tPoint(0.5, 0.3));
  control_points2.push_back(tPoint(0.6, 0.7));
  control_points2.push_back(tPoint(0.4, 0.9));

  tBezierCurve bezier_curve2(control_points2);

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



  window.Render();

  tWindow::ReleaseAllInstances();

  return EXIT_SUCCESS;
}
