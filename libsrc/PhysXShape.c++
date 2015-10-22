
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

#include "PhysXShape.h++"


PhysXShape::PhysXShape(atInt * id)
{
   // Initialize the unique identifier of this shape
   shape_id = id;

   // Since no shape has been supplied, initialize it to NULL
   physx_shape = NULL;
   shape_density = 0.0f;
}


PhysXShape::PhysXShape(atInt * id, PxShape * shape)
{
   // Initialize the unique identifier and the PhysX shape
   shape_id = id;
   physx_shape = shape;

   // Initialize the density to its default value
   shape_density = 0.0f;
}


PhysXShape::PhysXShape(atInt * id, PxShape * shape, float density)
{
   // Initialize the unique identifier, PhysX shape and density
   shape_id = id;
   physx_shape = shape;

   // Check to make sure the given density value is valid
   if (density >= 0.0)
      shape_density = density;
}


PhysXShape::~PhysXShape()
{
   // Check to see if this container has a valid PhysX shape
   if (physx_shape != NULL)
   {
      // Release this container's reference to the PhysX shape
      physx_shape->release();
   }
}


atInt * PhysXShape::getID()
{
   // Return the unique identifier of this shape
   return shape_id;
}


PxShape * PhysXShape::getShape()
{
   // Return the PhysX shape represented by this container
   return physx_shape;
}


void PhysXShape::setShape(PxShape * shape)
{
   // Check to see if this container already has a shape
   if (physx_shape != NULL)
   {
      // Release the reference to the old shape
      physx_shape->release();
   }

   // Update the PhysX shape represented by this container
   physx_shape = shape;
}


float PhysXShape::getDensity()
{
   // Return the density of the shape
   return shape_density;
}


void PhysXShape::setDensity(float density)
{
   // Update the density of the shape as long it is valid
   if (density >= 0.0)
      shape_density = density;
}

