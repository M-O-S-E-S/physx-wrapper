
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


#ifndef PHYSX_ACTOR_H
#define PHYSX_ACTOR_H

#include "PxPhysicsAPI.h"

#include "atInt.h++"
#include "atItem.h++"
#include "atMap.h++"
#include "atString.h++"


using namespace physx;

/// Enumeration to determine if an actor should be static in the physical
/// scene or dynamic.
///
enum ActorType
{
   STATIC = 0,
   DYNAMIC = 1
};


/// Container class to help with managing the PhysX actors.

class PhysXRigidActor : public atItem
{
   protected:
      /// Current PhysX actor reference.
      ///
      PxRigidActor *     rigid_actor;

      /// The shapes that are attached to this actor and represent it in the
      /// physical scene.
      ///
      atMap *            actor_shapes;

      /// The name of this actor.
      ///
      atString *         actor_name;

      /// The identifier of this PhysX object used to quickly locate the actor
      /// inside of maps.
      ///
      atInt *            actor_id;

      /// Stores whether this actor is dynamic or static actor.
      ///
      ActorType          actor_type;

      /// Updates the densities associated PhysX actor based on the densities
      /// of the various shapes attached to the actor.
      ///
      void               updateDensity();

      /// The default density used when a given density is invalid
      ///
      static const float   default_density;

   public:
      /// Constructor
      ///
      /// @param physics Reference to the PhysX physics for use in calling 
      /// PhysX classes.
      /// 
      /// @param id The id for this rigid actor.
      /// @param x The x value of the position of this actor inside of the 
      /// physical scene.
      /// @param y The y value of the position of this actor inside of the 
      /// physical scene.
      /// @param z The z value of the position of this actor inside of the 
      /// physical scene.
      /// @param type This is the enumeration value to determine if the rigid
      /// actor is static or dynamic.
      ///
      PhysXRigidActor(PxPhysics * physics, unsigned int id,
         float x, float y, float z, ActorType type);
      
      /// Constructor
      ///
      /// @param physics Reference to the PhysX physics for use in calling 
      /// PhysX classes.
      /// @param id The id for this rigid actor.
      /// @param x The x value of the position of this actor inside of the 
      /// physical scene.
      /// @param y The y value of the position of this actor inside of the 
      /// physical scene.
      /// @param z The z value of the position of this actor inside of the 
      /// physical scene.
      /// @param rot A quaternion that represents the current orientation of
      /// the actor.
      /// @param type This is the enumeration value to determine if the rigid
      /// actor is static or dynamic.
      ///
      PhysXRigidActor(PxPhysics * physics, unsigned int id,
         float x, float y, float z, PxQuat rot, ActorType type);
      
      /// Destructor
      ///
      ~PhysXRigidActor();

      /// Method to determine if this actor is dynamic or static.
      ///
      /// @return True if the actor is dynamic false if the actor is static.
      ///
      bool             isDynamic();

      /// Sets the identifier for this actor.
      /// 
      /// @param id The new identifier for this actor.
      ///
      void   setID(unsigned int id);
      
      /// Method to fetch the current identifier for this actor.
      ///
      /// @return The atInt object's get value will be the identifier for this 
      /// object. 
      atInt *   getID();

      /// Method to get the PhysX representation of this actor as a
      /// PxActor instance.
      ///
      /// @return The object that represents the PhysX actor inside of the 
      /// PhysX scene.
      ///
      PxActor *   getActor();

      /// Method to get the PhysX representation of this actor as a
      /// PxRigidActor instance.
      ///
      /// @return The object that represents the PhysX actor inside of the 
      /// PhysX scene.
      PxRigidActor *   getRigidActor();

      /// Method to attach a shape to the PhysX actor.
      ///
      /// @param shapeId The unique identifier of the shape being attached.
      /// @param shape The shape that is being attached to the actor.
      /// @param density The density of the shape
      ///
      void             addShape(unsigned int shapeId, PxShape * shape,
                                float density);
      
      /// Fetches a shape attached to the actor.
      ///
      /// @param The unique identifier of the desired shape.
      /// @return The PhysX shape attached to the actor with the given ID.
      /// Will be NULL if no shape was found with the given ID.
      ///
      PxShape *        getShape(unsigned int shapeId);

      /// Updates the density of a given shape.
      ///
      /// @param shapeId The unique identifier of the desired shape.
      /// @param density The new density of the shape.
      ///
      void             setShapeDensity(unsigned int shapeId, float density);

      /// Detaches and deletes a shape currently attached to the actor.
      ///
      /// @param shapeId The unique identifier of the shape to be removed.
      /// @remarks Note that the last shape of a PhysX static actor cannot be 
      /// removed.
      ///
      void             detachShape(unsigned int shapeId);

      /// Detaches and deletes all shapes currently attached to the actor.
      ///
      /// @remarks Note that the last shape of a PhysX static actor cannot be
      /// removed.
      ///
      void             detachAllShapes();

      /// Changes the name of the actor.
      ///
      /// @param name The new name of the actor.
      ///
      void             setName(char * name);
      
      /// Fetch the current name of the actor.
      ///
      /// @return The current name of the actor.
      ///
      atString *       getName();

      /// Get the mass for an actor (only valid for dynamic actors).
      ///
      /// @return float The mass of the physics actor.
      ///
      float   getMass();

      /// Applies a force (or impulse) to the actor as set by PhysX flags.
      ///
      /// @return bool If the force was applied.
      ///
      bool   addForce(PxVec3 vector);

      /// Changes the position and orientation of the actor.
      ///
      /// @param posX The x value of the new position for this actor.
      /// @param posY The y value of the new position for this actor.
      /// @param posZ The z value of the new position for this actor.
      /// @param rotX The x value of the quaternion that represents the new 
      /// orientation of this actor.
      /// @param rotY The y value of the quaternion that represents the new 
      /// orientation of this actor.
      /// @param rotZ The z value of the quaternion that represents the new 
      /// orientation of this actor.
      /// @param rotW The w value of the quaternion that represents the new 
      /// orientation of this actor.
      ///
      void   setTranslation(float posX, float posY, float posZ,
                          float rotX, float rotY, float rotZ, float rotW);

      /// Changes the position of the actor.
      ///
      /// @param x The x value of the new position for this actor.
      /// @param y The y value of the new position for this actor.
      /// @param z The z value of the new position for this actor.
      ///
      void   setPosition(float x, float y, float z);
      
      /// Fetch the current position of this actor.
      ///
      /// @return An array with the current position of the actor. The values
      /// of the array are [x, y, z].
      ///
      float *   getPosition();

      /// Changes the orientation of this actor.
      ///
      /// @param x The x value of the quaternion that represents the new 
      /// orientation of this actor.
      /// @param y The y value of the quaternion that represents the new 
      /// orientation of this actor.
      /// @param z The z value of the quaternion that represents the new 
      /// orientation of this actor.
      /// @param w The w value of the quaternion that represents the new 
      /// orientation of this actor.
      ///
      void   setRotation(float x, float y, float z, float w);

      /// Fetch the current orientation of this actor.
      ///
      /// @return An array with the current quaternion that represents the 
      /// orientation for this actor. The values of the array are [x, y, z, w].
      ///
      float *   getRotation();

      /// Changes the linear velocity for this actor.
      ///
      /// @param x The new velocity in the x direction for this actor.
      /// @param y The new velocity in the y direction for this actor.
      /// @param z The new velocity in the z direction for this actor.
      ///
      void   setLinearVelocity(float x, float y, float z);

      /// Changes the angular velocity for this actor.
      ///
      /// @param x The new velocity in the x direction for this actor.
      /// @param y The new velocity in the y direction for this actor.
      /// @param z The new velocity in the z direction for this actor.
      ///
      void   setAngularVelocity(float x, float y, float z);

      /// Changes the direction of gravity acting on this actor.
      ///
      /// @param x The new amount of gravity in the x direction acting on this 
      /// actor.
      /// @param y The new amount of gravity in the y direction acting on this 
      /// actor.
      /// @param z The new amount of gravity in the z direction acting on this 
      /// actor.
      ///
      void   setGravity(float x, float y, float z);

      /// Changes whether gravity is currently acting on this actor.
      ///
      /// @param enabled Flag that when true causes gravity to act upon the 
      /// actor and when false to not act upon the actor.
      ///
      void             enableGravity(bool enabled);

      /// Set the density for an actor (only valid for dynamic actors).
      ///
      /// @param mass The mass of the actor.
      /// @return bool Whether the update was successful or not.
      ///
      bool             setMass(float mass);
};


#endif

