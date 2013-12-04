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
/*!\file    tShape.hpp
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
// tShape constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tShape<Tdimension, TElement>::tShape()
  : changed(false),
    bounding_box(nullptr),
    center_of_gravity(nullptr)
{}

//----------------------------------------------------------------------
// tShape copy constructor
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tShape<Tdimension, TElement>::tShape(const tShape &other)
  : changed(false),
    bounding_box(nullptr),
    center_of_gravity(nullptr)
{}

//----------------------------------------------------------------------
// tShape assignment operator
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tShape<Tdimension, TElement> &tShape<Tdimension, TElement>::operator= (const tShape &other)
{
  this->changed = false;
  this->bounding_box = nullptr;
  this->center_of_gravity = nullptr;
  return *this;
}

//----------------------------------------------------------------------
// tShape destructor
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tShape<Tdimension, TElement>::~tShape()
{
  this->ResetMetaInformation();
}

//----------------------------------------------------------------------
// tShape BoundingBox
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tShape<Tdimension, TElement>::tBoundingBox &tShape<Tdimension, TElement>::BoundingBox() const
{
  if (this->changed)
  {
    this->ResetMetaInformation();
  }
  if (!this->bounding_box)
  {
    this->bounding_box = new tBoundingBox();
    this->UpdateBoundingBox(*this->bounding_box);
  }
  return *this->bounding_box;
}

//----------------------------------------------------------------------
// tShape CenterOfGravity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const typename tShape<Tdimension, TElement>::tPoint &tShape<Tdimension, TElement>::CenterOfGravity() const
{
  if (this->changed)
  {
    this->ResetMetaInformation();
  }
  if (!this->center_of_gravity)
  {
    this->center_of_gravity = new tPoint();
    this->UpdateCenterOfGravity(*this->center_of_gravity);
  }
  return *this->center_of_gravity;
}

//----------------------------------------------------------------------
// tShape SetChanged
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tShape<Tdimension, TElement>::SetChanged()
{
  this->changed = true;
}

//----------------------------------------------------------------------
// tShape ResetMetaInformation
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tShape<Tdimension, TElement>::ResetMetaInformation() const
{
  this->changed = false;
  delete this->bounding_box;
  this->bounding_box = nullptr;
  delete this->center_of_gravity;
  this->center_of_gravity = nullptr;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
