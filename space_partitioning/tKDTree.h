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
/*!\file    tKDTree.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-11-28
 *
 * \brief   Contains tKDTree
 *
 * \b tKDTree
 *
 * A kd-tree is a binary search-tree which nodes split a k-dimensional
 * search space a one of the k axis. Thus, a 2d-tree that splits always
 * in the middle of alternating axis yields a quad-tree. Its superior
 * flexibility comes from simplifying the dimensionality and not being
 * restricted to using always the middle of the split axis.
 */
//----------------------------------------------------------------------
#ifndef __rrlib__geometry__space_partitioning__tKDTree_h__
#define __rrlib__geometry__space_partitioning__tKDTree_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/geometry/tPoint.h"
#include "rrlib/geometry/tBoundingBox.h"

//----------------------------------------------------------------------
// Debugging
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
//! The kd-tree template class
/*!
 * This class implements a very generic kd-tree for arbitrary dimension
 * and input-data. Therefore, it is a template that takes the following
 * parameters:
 *
 * \param Tdimension     The dimension of this tree (the \e k in kd-tree)
 * \param T              The input data (see the following note)
 * \param TContentType   The optional specification of the content type of the input data
 *
 * \note
 * The input data \a T must behave like a container-class in the way of
 * providing access to the relevant data for this search tree. The part
 * of the input data that is processed within this search tree has to
 * allow an interpretation as k-dimension coordinates in the k-dimensional
 * search space. Therefore, \a T must implement
 * \code
 * const TContentType operator [] (size_t i) const;
 * \endcode
 * for \c i in [0, \a Tdimension).\n
 * If \a T additionally provides \a TContentType as a public \c typedef
 * \c T::tContentType (like rrlib::math::tVector does) the third template
 * parameter can be omitted.
 *
 * This allows the usage of tKDTree like the following examples show:
 * \code
 * // the data to create a 2d-tree for is a list of simple 2d-float-arrays
 * std::vector<float[2]> data;
 * // ... fill data
 * tKDTree<2, float[2], float> kd_tree(data);
 * \endcode
 * \code
 * // the data to create a 2d-tree for is a list of simple double-pointers to 4d-arrays
 * std::vector<double *> data;
 * // ... fill data
 * tKDTree<4, double *, double> kd_tree(data);
 * \endcode
 * \code
 * // even more complex data structures can be used
 * class tKDTreeExample
 * {
 *
 * public:
 *
 *   tKDTreeExample()
 *   {
 *     std::vector<tData> data;
 *     // ... fill data
 *     tKDTree<3, tData> kd_tree(data);
 *   }
 *
 * private:
 *
 *   struct tData
 *   {
 *     typedef rrlib::math::tVec3d::tContentType tContentType;
 *
 *     rrlib::math::tVec3d data;
 *     int some_other_information;
 * //    tSomeType *more_other_information;
 *
 *     const tContentType operator [] (size_t i) const { return data[i]; }
 *   };
 *
 * };
 * \endcode
 */
template <size_t Tdimension, typename TElement>
class tKDTree
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef geometry::tPoint<Tdimension, TElement> tPoint;
  typedef geometry::tBoundingBox<Tdimension, TElement> tBoundingBox;
  typedef std::function < TElement(const tPoint &, const tPoint &) > tMetric;

  /*!
   * \brief An inner class of the tKDTree template for the nodes of the tree
   *
   * This class is used to store the structure of the tree and is public to
   * provide an interface to the information stored in the nodes.
   *
   * It stores the number of points that are grouped within this node, the
   * minimum and maximum of the points' coordinates (can be used to define
   * an axis-aligned bounding box), the center of mass of the points rooted
   * at this node.
   */
  class tNode
  {

    //----------------------------------------------------------------------
    // Public methods and typedefs
    //----------------------------------------------------------------------
  public:

    /*!
     * \brief The ctor of tNode
     *
     * The tNode gets a part of a \c std::vector<tConstInputIterator>
     * defined by \a begin and \a end to manage as points below itself. It
     * therefore splits this list and propagates the parts to nodes it creates
     * as its own children until only single points are left.
     *
     * \param begin    An indirect iterator to the begin of the data to manage
     * \param end      An indirect iterator to the end of the data to manage
     * \param metric   A functor that computes an appropriate metric
     */
    template <typename TIterator>
    tNode(TIterator begin, TIterator end, tMetric metric);

    /*!
     * brief The dtor of tKDTreeNode
     */
    ~tNode();

    /*!
     * \brief Get the split-axis of this node
     *
     * \return The index of the axis
     */
    inline size_t SplitAxis() const;

    /*!
     * \brief Get the split-value of this node
     *
     * \return The value that defines the split point \f$s: x < s \rightarrow x\f$ can be found in the left child-node
     */
    inline TElement SplitValue() const;

    /*!
     * \brief Get the number of points rooted at this node
     *
     * \return The number of points that can be found underneath this node
     */
    inline size_t NumberOfPoints() const;

    /*!
     * \brief Get the classification of this node being a leaf or not
     *
     * \return Whether this node is a leaf (\c true) or not (\c false)
     */
    inline bool IsLeaf() const;

    /*!
     * \brief Get the axis-aligned boungind box of the points underneath this node
     *
     * \return The maximum coordinates of this node
     */
    inline const tBoundingBox &BoundingBox() const;

    /*!
     * \brief Get the center of mass of the points underneath this node
     *
     * \return The center of mass of this node
     */
    inline const tPoint &CenterOfMass() const;

    /*!
     * \brief Get the left child of this node in the tree
     *
     * \return The left child of this node
     */
    inline const tNode &LeftChild() const;

    /*!
     * \brief Get the right child of this node in the tree
     *
     * \return The right child of this node
     */
    inline const tNode &RightChild() const;

    //----------------------------------------------------------------------
    // Private fields and methods
    //----------------------------------------------------------------------
  private:

    tBoundingBox bounding_box;

    size_t split_axis;
    TElement split_value;
    tNode *left_child;
    tNode *right_child;

    size_t number_of_points;
    tPoint center_of_mass;

    size_t SelectSplitAxis(tMetric metric) const;
  };

  /*!
   * \brief The ctor of tKDTree
   *
   * The tKDTree creates a tree structure for a given point-set \a points.
   * Therefore it first generates a list of iterators to the elements in
   * \a data to benefit from the generic and fast interface instead of
   * reordering \a data when sorting operations have to be performed.
   *
   * The next step is to create the root of the tree with to complete
   * range of input data and let the ctor of tKDTreeNode create the
   * complete tree in a recursive fashion.
   *
   * \param data     The data to organize in a kd-tree
   * \param metric   A functor that computes an appropriate metric
   *
   * \warning
   * The kd-tree is a static structure and has to be rebuild after changes
   * in the underlying data.
   */
  template <typename TIterator>
  tKDTree(TIterator begin, TIterator end);

  template <typename TIterator>
  tKDTree(TIterator begin, TIterator end, tMetric metric);

  /*!
   * \brief The dtor of tKDTree
   */
  ~tKDTree();

  /*!
   * \brief Get the root of the tree to start traversal through the structure
   *
   * \return The uppermost node in the tree that contains all points
   */
  inline const tNode &Root() const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tNode *root;

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/geometry/space_partitioning/tKDTree.hpp"

#endif
