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
template <size_t Tdimension, typename TElement>
const size_t tShape<Tdimension, TElement>::cDIMENSION = Tdimension;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tShape constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
tShape<Tdimension, TElement>::tShape()
    : changed(false),
    bounding_box(NULL),
    center_of_gravity(NULL)
{}

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
// tShape ResetMetaInformation
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
void tShape<Tdimension, TElement>::ResetMetaInformation() const
{
  this->changed = false;
  delete this->bounding_box;
  this->bounding_box = NULL;
  delete this->center_of_gravity;
  this->center_of_gravity = NULL;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}