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
/*!\file    rrlib/geometry/tests/geometry.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2017-02-17
 *
 * Tests of diverse geometric operations
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tUnitTestSuite.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tLine.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace geometry
{
namespace tests
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
class TestGeometry : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(TestGeometry);
  RRLIB_UNIT_TESTS_ADD_TEST(LineSegmentBoundingBoxIntersectionTest);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests() override
  {
  }

  virtual void CleanUp() override
  {
  }

  template <size_t Tdimension, typename TElement>
  static void AssertLineSegmentsEqual(const tLineSegment<Tdimension, TElement>& actual, const tLineSegment<Tdimension, TElement>& expected)
  {
    for (size_t i = 0; i < Tdimension; i++)
    {
      RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(expected.Begin()[i], actual.Begin()[i], 0.0001);
      RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(expected.End()[i], actual.End()[i], 0.0001);
    }
  }

  void LineSegmentBoundingBoxIntersectionTest()
  {
    // Multiple times the same test with differenct x offsets to detect numeric instabilities
    LineSegmentBoundingBoxIntersectionTestHelper(0);
    LineSegmentBoundingBoxIntersectionTestHelper(1);
    for (int i = 400000000; i < 400000100; i++)
    {
      LineSegmentBoundingBoxIntersectionTestHelper(i);
    }
  }

  void LineSegmentBoundingBoxIntersectionTestHelper(double x)
  {
    {
      tBoundingBox<2, double> box;
      typedef tLineSegment2D tLineSegment;
      typedef tLineSegment::tPoint tPoint;
      box.Add(tPoint(-4 + x, -1));
      box.Add(tPoint(-2 + x, 9));
      tLineSegment inner(tPoint(-2 + x, 8), tPoint(-3 + x, 7));
      tLineSegment inner_rev(tPoint(-3 + x, 7), tPoint(-2 + x, 8));
      AssertLineSegmentsEqual(inner.GetIntersection(box).second, inner);
      AssertLineSegmentsEqual(inner_rev.GetIntersection(box).second, inner_rev);
      RRLIB_UNIT_TESTS_ASSERT(!tLineSegment(tPoint(-7 + x, -11), tPoint(-9 + x, 11)).GetIntersection(box).first);

      // Check diagonals
      AssertLineSegmentsEqual(tLineSegment(tPoint(0 + x, -11), tPoint(-6 + x, 19)).GetIntersection(box).second, tLineSegment(tPoint(-2 + x, -1), tPoint(-4 + x, 9)));
      AssertLineSegmentsEqual(tLineSegment(tPoint(-6 + x, -11), tPoint(0 + x, 19)).GetIntersection(box).second, tLineSegment(tPoint(-4 + x, -1), tPoint(-2 + x, 9)));

      // Check ray through both sides of one box
      AssertLineSegmentsEqual(tLineSegment(tPoint(-3 + x, 0), tPoint(-3 + x, 10)).GetIntersection(box).second, tLineSegment(tPoint(-3 + x, 0), tPoint(-3 + x, 9)));
    }
    {
      tBoundingBox<3, double> box;
      typedef tLineSegment3D tLineSegment;
      typedef tLineSegment::tPoint tPoint;

      box.Add(tPoint(-2 + x, 3, -1));
      box.Add(tPoint(1 + x, 5, 4));

      AssertLineSegmentsEqual(tLineSegment(tPoint(-1 + x, 4, 5), tPoint(2 + x, 4, 2)).GetIntersection(box).second, tLineSegment(tPoint(0 + x, 4, 4), tPoint(1 + x, 4, 3)));
      AssertLineSegmentsEqual(tLineSegment(tPoint(0 + x, 4, 4), tPoint(12 + x, 5, 8)).GetIntersection(box).second, tLineSegment(tPoint(0 + x, 4, 4), tPoint(0 + x, 4, 4)));

      // Check line segments
      AssertLineSegmentsEqual(tLine<3, double>(tPoint(-111 + x, 4, 115), rrlib::math::tVec3d(-1, 0, 1)).GetIntersection(box).second, tLineSegment(tPoint(1 + x, 4, 3), tPoint(0 + x, 4, 4)));
    }
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(TestGeometry);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
