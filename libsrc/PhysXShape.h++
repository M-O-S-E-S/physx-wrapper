
// PhysX Wrapper
//
// Copyright 2015 University of Central Florida
//
//
// This library wraps up the native calls to NVIDIA's PhysX API and
// provides higher-level functions to C# for use.
//
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef PHYSX_SHAPE_H
#define PHYSX_SHAPE_H

#include "PxPhysicsAPI.h"

#include "atInt.h++"
#include "atItem.h++"


using namespace physx;


/// Container class for managing PhysX shapes.

class PhysXShape : public atItem
{
   protected:
      /// Reference to the PhysX shape.
      ///
      PxShape *   physx_shape;

      /// The unique identifier of this shape.
      ///
      atInt *   shape_id;

      /// The density of the shape.
      ///
      float   shape_density;

   public:
      /// Constructor.
      ///
      /// @param id The unique identifier of the shape.
      ///
      PhysXShape(atInt * id);

      /// Constructor.
      ///
      /// @param id The unique identifier of the shape.
      /// @param shape The PhysX shape represented by this container.
      ///
      PhysXShape(atInt * id, PxShape * shape);

      /// Constructor.
      ///
      /// @param id The unique identifier of the shape.
      /// @param shape The PhysX shape represented by this container.
      /// @param density The density of the shape.
      ///
      PhysXShape(atInt * id, PxShape * shape, float density);

      /// Destructor.
      ///
      ~PhysXShape();

      /// Returns the unique identifier of the shape.
      ///
      /// @return The unique identifier of the shape.
      ///
      atInt *   getID();

      /// Returns the PhysX shape associated with this shape.
      ///
      /// @return The PhysX shape associated with this shape.
      ///
      PxShape *   getShape();

      /// Updates the PhysX shape associated with this shape.
      ///
      /// @param shape The PhysX shape to be represented by this container.
      ///
      void   setShape(PxShape * shape);

      /// Returns the density of the shape.
      ///
      /// @return The density of the shape.
      ///
      float   getDensity();

      /// Updates the density of the shape.
      ///
      /// @param density The new density of the shape.
      ///
      void   setDensity(float density);
};

#endif

