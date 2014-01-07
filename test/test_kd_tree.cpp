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
/*!\file    test_kd_tree.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2012-01-08
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>
#include <iostream>
#include <vector>

#include <random>
#include <functional>

#include "rrlib/highgui_wrapper/tWindow.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tPoint.h"
#include "rrlib/geometry/space_partitioning/tKDTree.h"

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
typedef rrlib::geometry::tPoint<2, tElement> tPoint;
typedef rrlib::geometry::tKDTree<2, tElement> tKDTree;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const unsigned int cWINDOW_SIZE  = 500;
const double cNORMAL_DISTRIBUTION_QUANTILE_95_PERCENT = 1.6449;
const double cNORMAL_DISTRIBUTION_QUANTILE_99_PERCENT = 2.3263;
const double cNORMAL_DISTRIBUTION_QUANTILE_99_9_PERCENT = 3.0902;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

struct tManhattanNorm
{
  inline tElement operator()(const tPoint &a, const tPoint &b) const
  {
    return std::fabs(a.X() - b.X()) + std::fabs(a.Y() - b.Y());
  }
};

int ManhattanNorm(const tPoint &a, const tPoint &b)
{
  return std::fabs(a.X() - b.X()) + std::fabs(a.Y() - b.Y());
}

auto manhattan_norm = [](const tPoint & a, const tPoint & b)
{
  return std::fabs(a.X() - b.X()) + std::fabs(a.Y() - b.Y());
};



void DrawPoint(tWindow &window, const tPoint &point)
{
  window.DrawCircle(point.X() + 0.5 * cWINDOW_SIZE, point.Y() + 0.5 * cWINDOW_SIZE, 1, true);
}

void DrawCircle(tWindow &window, const tPoint &center, tElement radius)
{
  window.DrawCircle(center.X() + 0.5 * cWINDOW_SIZE, center.Y() + 0.5 * cWINDOW_SIZE, radius, false);
}

void DrawPoints(tWindow &window, const std::vector<tPoint> &points)
{
  for (auto it = points.begin(); it != points.end(); ++it)
  {
    DrawPoint(window, *it);
  }
}

void DrawKDTreeNode(tWindow &window, const tKDTree::tNode &node, int start, int stop, int level)
{
  if (level > stop)
  {
    return;
  }

  if (start <= level)
  {
    window.DrawRectangle(node.BoundingBox().Min().X() + 0.5 * cWINDOW_SIZE, node.BoundingBox().Min().Y() + 0.5 * cWINDOW_SIZE,
                         node.BoundingBox().Max().X() + 0.5 * cWINDOW_SIZE, node.BoundingBox().Max().Y() + 0.5 * cWINDOW_SIZE);
  }

  if (!node.IsLeaf())
  {
    DrawKDTreeNode(window, node.LeftChild(), start, stop, level + 1);
    DrawKDTreeNode(window, node.RightChild(), start, stop, level + 1);
  }
}

void DrawKDTree(tWindow &window, const tKDTree &kd_tree, int start, int stop)
{
  DrawKDTreeNode(window, kd_tree.Root(), start, stop, 0);
}

void GenerateRandomClusteredPoints(std::vector<tPoint> &points,
                                   unsigned int min_number_of_samples, unsigned int max_number_of_samples,
                                   unsigned int min_number_of_clusters, unsigned int max_number_of_clusters,
                                   unsigned int min_cluster_radius, unsigned int max_cluster_radius,
                                   double normal_distribution_quantile_factor, unsigned long seed = time(NULL))
{
  points.clear();

  std::mt19937 rng_engine(seed);

  unsigned int number_of_clusters = std::uniform_int_distribution<unsigned int>(min_number_of_clusters, max_number_of_clusters)(rng_engine);

  struct tCluster
  {
    tPoint center;
    tElement radius;
    std::normal_distribution<tElement> distribution_x;
    std::normal_distribution<tElement> distribution_y;
  };
  std::vector<tCluster> clusters(number_of_clusters);

  auto random_cluster_center_component = std::bind(std::uniform_real_distribution<tElement>(-0.5 * cWINDOW_SIZE, 0.5 * cWINDOW_SIZE), rng_engine);
  auto random_cluster_radius = std::bind(std::uniform_real_distribution<tElement>(min_cluster_radius, max_cluster_radius), rng_engine);
  for (size_t i = 0; i < number_of_clusters; ++i)
  {
    clusters[i].center.Set(random_cluster_center_component(), random_cluster_center_component());
    clusters[i].radius = random_cluster_radius();
    for (size_t k = 0; k < i; ++k)
    {
      if ((clusters[i].center - clusters[k].center).Length() < 2 * max_cluster_radius)
      {
        --i;
        break;
      }
    }
  }

  tWindow &window(tWindow::GetInstance("Test KD-Tree"));
  window.SetColor(1);
  for (size_t i = 0; i < number_of_clusters; ++i)
  {
    DrawPoint(window, clusters[i].center);
    DrawCircle(window, clusters[i].center, clusters[i].radius);
  }
  window.Render();

  for (size_t i = 0; i < number_of_clusters; ++i)
  {
    clusters[i].distribution_x = std::normal_distribution<tElement>(clusters[i].center.X(), clusters[i].radius / normal_distribution_quantile_factor);
    clusters[i].distribution_y = std::normal_distribution<tElement>(clusters[i].center.Y(), clusters[i].radius / normal_distribution_quantile_factor);
  }

  unsigned int number_of_samples = std::uniform_int_distribution<unsigned int>(min_number_of_samples, max_number_of_samples)(rng_engine);

  for (unsigned int i = 0; i < number_of_samples; ++i)
  {
    size_t cluster_id = std::uniform_int_distribution<size_t>(0, number_of_clusters - 1)(rng_engine);
    points.push_back(tPoint(clusters[cluster_id].distribution_x(rng_engine), clusters[cluster_id].distribution_y(rng_engine)));
  }

  window.SetColor(2);
  for (auto it = points.begin(); it < points.end(); ++it)
  {
    DrawPoint(window, *it);
  }
  window.Render();
}

int main(int argc, char **argv)
{
  tWindow &window(tWindow::GetInstance("Test KD-Tree", cWINDOW_SIZE, cWINDOW_SIZE));

  std::vector<tPoint> points;

  GenerateRandomClusteredPoints(points, 2000, 4000, 3, 15, 20, 60, cNORMAL_DISTRIBUTION_QUANTILE_95_PERCENT);

  window.Clear();
  DrawPoints(window, points);
  window.Render();

  tKDTree kd_tree(points.begin(), points.end());
//  tKDTree kd_tree(points.begin(), points.end(), [](const tPoint &a, const tPoint &b){return (a - b).Length();});
//  tKDTree kd_tree(points.begin(), points.end(), tManhattanNorm());
//  tKDTree kd_tree(points.begin(), points.end(), ManhattanNorm);
//  tKDTree kd_tree(points.begin(), points.end(), manhattan_norm);

  for (unsigned int i = 0; i < 10; i++)
  {
    window.Clear();
    DrawPoints(window, points);
    window.SetColor(0, 0, 255);
    DrawKDTree(window, kd_tree, i, i);
    std::cout << std::endl << "Showing level " << i << " of kd-tree" << std::endl;
    window.Render();
  }

  window.Clear();
  DrawPoints(window, points);
  window.SetColor(0, 0, 255);
  DrawKDTree(window, kd_tree, 0, 7);
  std::cout << std::endl << "These are the first 7 levels of the kd-tree" << std::endl;
  window.Render();

  tWindow::ReleaseAllInstances();

  return EXIT_SUCCESS;
}
