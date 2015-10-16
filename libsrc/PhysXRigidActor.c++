
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


#include "PhysXRigidActor.h++"


PhysXRigidActor::PhysXRigidActor(PxPhysics * physics, unsigned int id,
   float x, float y, float z, ActorType type)
{
   PxTransform   actorPos;

   // Set the name of the class for the user notification messages
   atNotifier::setName("[PhysXRigidActor] ");

   // Create the position transform for this actor
   actorPos = PxTransform(PxVec3(x, y, z));

   // Determine if the actor is dynamic or static
   if (type == DYNAMIC)
   {
      // Create a dynamic actor for physical interactions
      rigid_actor = (PxRigidActor *) physics->createRigidDynamic(actorPos);

      // Enable continuous collision detection for this dynamic actor
      ((PxRigidBody *) rigid_actor)->setRigidBodyFlag(
         PxRigidBodyFlag::eENABLE_CCD, true);
   }
   else
   {
      // Create a static actor for physical interactions
      rigid_actor = (PxRigidActor *) physics->createRigidStatic(actorPos);
   }

   // Don't assign a name yet 
   actor_name = NULL;

   // Store the current identifier that will be used in maps to quickly access
   // this actor
   actor_id = new atInt(id);

   // Store the type of actor that was created either static or dynamic
   actor_type = type;

   // Assign the ID to the actor's user data
   rigid_actor->userData = (void *)actor_id;
}


PhysXRigidActor::PhysXRigidActor(PxPhysics * physics, unsigned int id,
   float x, float y, float z, PxQuat rot, ActorType type)
{
   PxTransform   actorPos;

   // Set the name of the class for the user notification messages
   setName("[PhysXRigidActor] ");

   // Create the position transform for this actor
   actorPos = PxTransform(PxVec3(x, y, z), rot);

   // Determine the type of actor that needs to be created for this actor
   if (type == DYNAMIC)
   {
      // Create a dynamic actor for physical interactions
      rigid_actor = (PxRigidActor *) physics->createRigidDynamic(actorPos);

      // Enable continuous collision detection for this dynamic actor
      ((PxRigidBody *) rigid_actor)->setRigidBodyFlag(
         PxRigidBodyFlag::eENABLE_CCD, true);
   }
   else
   {
      // Create a static actor for physical interactions
      rigid_actor = (PxRigidActor *) physics->createRigidStatic(actorPos);
   }

   // Don't assign a name yet
   actor_name = NULL;

   // Store the current identifier that will be used in maps to quickly access
   // this actor
   actor_id = new atInt(id);

   // Store the type of actor that was created either static or dynamic
   actor_type = type;

   // Assign the ID to the actor's user data
   rigid_actor->userData = (void *)actor_id;
}


PhysXRigidActor::~PhysXRigidActor()
{
   // Clean up the PxActor; this will also delete the actor
   // ID since it's reference is kept in the actor's user data
   if (rigid_actor != NULL)
      rigid_actor->release();

   // Remove the actor name
   if (actor_name != NULL)
      delete actor_name;
}


void PhysXRigidActor::setID(unsigned int id)
{
   // Remove the old ID if one existed
   if (actor_id != NULL)
      delete actor_id;

   // Save the new identifier to an atInt in order to use the atMap
   actor_id = new atInt(id);
}


atInt * PhysXRigidActor::getID()
{
   // Return the current identifier for this actor which is used as a key for
   // maps
   return actor_id;
}


PxActor * PhysXRigidActor::getActor()
{
   // Return the current PhysX actor
   return ((PxActor *)rigid_actor);
}


PxRigidActor * PhysXRigidActor::getRigidActor()
{
   // Return the current PhysX rigid actor
   return rigid_actor;
}


void PhysXRigidActor::setShape(PxShape * shape)
{
   // Attach the new shape to the PhysX actor
   rigid_actor->attachShape(*shape);

   // Store the new shape to the instance field
   actor_shape = shape;
}


PxShape * PhysXRigidActor::getShape()
{
   // Return the current shape of this object
   return actor_shape;
}


void PhysXRigidActor::setName(char * name)
{
   // Remove our previous name if one existed
   if (actor_name != NULL)
      delete actor_name;

   // Save the new name to our atString field
   actor_name = new atString(name);

   // Update PhysX actor with the new name
   rigid_actor->setName(actor_name->getString());
}


atString * PhysXRigidActor::getName()
{
   // Return the current name of this actor
   return actor_name;
}


bool PhysXRigidActor::setDensity(float density)
{
   if (actor_type == DYNAMIC)
   {
       return PxRigidBodyExt::updateMassAndInertia(
                 *((PxRigidDynamic *) rigid_actor), density);
   }

   // Not allowed to set density on non-dynamic actors
   return false;
}


bool PhysXRigidActor::setMass(float mass)
{
   // Only dynamic actors can have a mass
   if (actor_type == DYNAMIC)
   {
       return PxRigidBodyExt::setMassAndUpdateInertia(
                 *((PxRigidDynamic *) rigid_actor), mass);
   }

   // Not allowed to set density on non-dynamic actors
   return false;
}


float PhysXRigidActor::getMass()
{
   // Only dynamic actors can have a mass
   if (actor_type == DYNAMIC)
   {  
      return ((PxRigidDynamic *) rigid_actor)->getMass();  
   }

   return 0.0f;
}

bool PhysXRigidActor::addForce(PxVec3 force)
{
   // Only dynamic actors should have forces applied
   if (actor_type == DYNAMIC)
   {
      ((PxRigidDynamic *) rigid_actor)->addForce(force);
      return true;
   }

   return false;
}


void PhysXRigidActor::setTranslation(float posX, float posY, float posZ,
   float rotX, float rotY, float rotZ, float rotW)
{
   PxVec3        position;
   PxQuat        quaternion;
   PxTransform   transform;

   // Create the new position and orientation for use with the PhysX transform
   // class
   position = PxVec3(posX, posY, posZ);
   quaternion = PxQuat(rotX, rotY, rotZ, rotW);

   // Create a new transform that uses the updated position and orientation
   transform = PxTransform(position, quaternion);

   // Change the global position and orientation to the new transform
   rigid_actor->setGlobalPose(transform);
}


void PhysXRigidActor::setPosition(float x, float y, float z)
{
   PxTransform   transform;
   PxQuat        quaternion;

   // Create the new position with the given values
   quaternion = rigid_actor->getGlobalPose().q;
   transform = PxTransform(PxVec3(x, y, z), quaternion);

   // Update the actor's position and indicate that it should be
   // woken up so that it can be updated during the simulation step
   rigid_actor->setGlobalPose(transform);
}


float * PhysXRigidActor::getPosition()
{
   PxVec3    position;
   float *   retValue;

   // Acquire the current position of this actor
   position = rigid_actor->getGlobalPose().p;

   // Store the position values so that PhysX does not change the values while
   // the information is being used
   retValue = new float[3];
   retValue[0] = position.x;
   retValue[1] = position.y;
   retValue[2] = position.z;

   // Return the current position of this actor
   return retValue;
}


void PhysXRigidActor::setRotation(float x, float y, float z, float w)
{
   PxTransform   transform;
   PxVec3        position;

   // Get the current position to prevent changes to the position of this actor
   // when the orientation changes
   position = rigid_actor->getGlobalPose().p;

   // Create a new transform with the updated orientation and original position
   transform = PxTransform(position, PxQuat(x, y, z, w));

   // Update the actor to use the new orientation
   rigid_actor->setGlobalPose(transform);
}


float * PhysXRigidActor::getRotation()
{
   PxQuat    quaternion;
   float *   retValue;

   // Acquire the current orienation of this actor
   quaternion = rigid_actor->getGlobalPose().q;

   // Store the orientation into a returnable value so that PhysX does not
   // change the values while the information is being used
   retValue = new float[4];
   retValue[0] = quaternion.x;
   retValue[1] = quaternion.y;
   retValue[2] = quaternion.z;
   retValue[3] = quaternion.w;

   // Return the current orientation of this actor
   return retValue;
}


void PhysXRigidActor::setLinearVelocity(float x, float y, float z)
{
   // Update the actor's linear velocity
   if (actor_type == DYNAMIC)
      ((PxRigidDynamic *) rigid_actor)->setLinearVelocity(PxVec3(x, y, z));
}


void PhysXRigidActor::setAngularVelocity(float x, float y, float z)
{
   // Update the actor's angular velocity
   if (actor_type == DYNAMIC)
   {
      ((PxRigidDynamic *) rigid_actor)->setAngularVelocity(PxVec3(x, y, z));
   }
}


void PhysXRigidActor::setGravity(float x, float y, float z)
{
   // TODO: Currently commands for changing the gravity are not handled by
   // opensim except to turn gravity on and off. A later commit will implement
   // this.
}


void PhysXRigidActor::enableGravity(bool enabled)
{
   // Update whether gravity should affect this actor, normally used for static
   // actors or flying avatars
   if (enabled)
   {
      rigid_actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
   }
   else
   {
      rigid_actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
   }
}


bool PhysXRigidActor::isDynamic()
{
   // Use the ActorType enumertation to determine if the actor is dynamic or
   // static
   if (actor_type == DYNAMIC)
      return true;
   else
      return false;
}

