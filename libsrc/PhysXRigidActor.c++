
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
#include "PhysXShape.h++"

#include "pthread.h"


// Initialize the default density that will be used in case a supplied density
// is invalid
const float PhysXRigidActor::default_density = 1000.0006836f;


PhysXRigidActor::PhysXRigidActor(PxPhysics * physics, unsigned int actorID,
   float x, float y, float z, ActorType type)
{
   PxTransform   actorPos;

   // Set the name of the class for the user notification messages
   atNotifier::setName("[PhysXRigidActor] ");

   // Initialize the actor mutex
   pthread_mutex_init(&actor_mutex, NULL);

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

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

   // Assign a blank name for now
   actor_name = new atString("");

   // Store the current identifier that will be used in maps to quickly access
   // this actor
   actor_id = new atInt(actorID);

   // Store the type of actor that was created either static or dynamic
   actor_type = type;

   // Assign the ID to the actor's user data
   rigid_actor->userData = actor_id;

   // Create the map that will hold the various shapes attached to this actor
   actor_shapes = new atMap();

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


PhysXRigidActor::PhysXRigidActor(PxPhysics * physics, unsigned int id,
   float x, float y, float z, PxQuat rot, ActorType type)
{
   PxTransform   actorPos;

   // Set the name of the class for the user notification messages
   setName("[PhysXRigidActor] ");

   // Initialize the actor mutex
   pthread_mutex_init(&actor_mutex, NULL);

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

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

   // Assign a blank name for now
   actor_name = new atString("");

   // Store the current identifier that will be used in maps to quickly access
   // this actor
   actor_id = new atInt(id);

   // Store the type of actor that was created either static or dynamic
   actor_type = type;

   // Assign the ID to the actor's user data
   rigid_actor->userData = actor_id;

   // Create the map that will hold the various shapes attached to this actor
   actor_shapes = new atMap();

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


PhysXRigidActor::~PhysXRigidActor()
{
   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Clean up the map containing the shape attached to this actor
   delete actor_shapes;

   // Clean up the ID of this actor
   delete actor_id;

   // Clean up the PxActor; this will also delete the actor
   // ID since it's reference is kept in the actor's user data
   if (rigid_actor != NULL)
      rigid_actor->release();

   // Remove the actor name
   if (actor_name != NULL)
      delete actor_name;

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Clean up the mutex for this actor
   pthread_mutex_destroy(&actor_mutex);
}


void PhysXRigidActor::setID(unsigned int id)
{
   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Set new ID value
   actor_id->setValue(id);

   // Assign the ID to the actor's user data, so that this actor can be
   // identified in PhysX callbacks
   rigid_actor->userData = actor_id;

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


unsigned int PhysXRigidActor::getID()
{
   unsigned int   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Return the current identifier for this actor which is used as a key for
   // maps
   result = actor_id->getValue();

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return the ID
   return result;
}


PxActor * PhysXRigidActor::getActor()
{
   PxActor *   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Return the current PhysX actor
   result = ((PxActor *)rigid_actor);

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return the PhysX actor
   return result;
}


PxRigidActor * PhysXRigidActor::getRigidActor()
{
   PxRigidActor *   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Return the current PhysX rigid actor
   result = rigid_actor;

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return the PhysX actor
   return result;
}


void PhysXRigidActor::addShape(unsigned int shapeId, PxShape * shape,
   float density)
{
   atInt *        tempId;
   PhysXShape *   newShape;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Convert the given shape ID into an atInt, so that it can be used with
   // the shape map
   tempId = new atInt(shapeId);

   // Check to see if the this actor has a shape already with the given ID
   if (actor_shapes->getValue(tempId) != NULL)
   {
      // Clean up the ID an exit, so that the existing shape is not overwritten
      notify(AT_WARN, "Failed to attach shape! Actor already contains shape "
         "with given ID (%u).\n", shapeId);
      delete tempId;
   }
   else
   {
      // Create a new shape container to hold the information for the shape
      newShape = new PhysXShape(new atInt(shapeId), shape, density);
 
      // Create an entry for the new shape in the shape map
      actor_shapes->addEntry(tempId, newShape);
 
      // Attach the new shape to the PhysX actor
      rigid_actor->attachShape(*shape);
 
      // Now that a new shape has been attached, the densities have to
      // be updated
      updateDensity();
   }

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Clean up and exit
   return;
}


PxShape * PhysXRigidActor::getShape(unsigned int shapeId)
{
   atInt *        tempId;
   PhysXShape *   shapeObj;
   PxShape *      result;

   // Convert the given ID into an atInt, so that it can be used to check the
   // map
   tempId = new atInt(shapeId);

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Retrieve the shape attached to this actor with the given ID
   shapeObj = (PhysXShape *) actor_shapes->getValue(tempId);

   // Clean up the temporary ID now that the shape has been retrieved
   delete tempId;

   // Check to see if a shape container was found with the given ID
   if (shapeObj != NULL)
   {
      // Return the PhysX shape associated by the shape container
      result = shapeObj->getShape();
   }
   else
   {
      // The shape container was not found, so return NULL
      result = NULL;
   }

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return the result from the operations above
   return result;
}


void PhysXRigidActor::setShapeDensity(unsigned int shapeId, float density)
{
   PhysXShape *   shapeObj;
   atInt *        tempId;

   // Convert the given ID into an atInt, so that it can be used to check
   // the map
   tempId = new atInt(shapeId);

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Retrieve the shape attached to this actor with the given ID
   shapeObj = (PhysXShape *) actor_shapes->getValue(tempId);

   // Check to see if a shape with the given ID was found
   if (shapeObj != NULL)
   {
      // Modify the density of the shape with the given value
      shapeObj->setDensity(density);
   }

   // Re-calculate the density of the actor
   updateDensity();

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Clean up the temporary ID now that the density has been updated
   delete tempId;
}


void PhysXRigidActor::detachShape(unsigned int shapeId)
{
   atInt *        tempId;
   PhysXShape *   shape;

   // Convert the given ID into an atInt, so that it can be used to search
   // the shape map
   tempId = new atInt(shapeId);

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Attempt to remove a shape from the shape map with the given ID
   shape = (PhysXShape *) actor_shapes->removeEntry(tempId);

   // Check to see if a shape was removed
   if (shape != NULL)
   {
      // Detach the PhysX shape from the actor
      rigid_actor->detachShape(*shape->getShape());

      // Clean up the shape container
      delete shape;
   }

   // Now that the shape has been detached, the densities have to be updated
   updateDensity();

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Clean up the temporary ID now that the operations are complete
   delete tempId;
}


void PhysXRigidActor::detachAllShapes()
{
   atList *       shapeIds;
   atList *       shapes;
   PhysXShape *   currShape;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Retrieve all the shapes that are attached to this actor
   shapeIds = new atList();
   shapes = new atList();
   actor_shapes->getSortedList(shapeIds, shapes);

   // Go through each of the shapes
   currShape = (PhysXShape *) shapes->getFirstEntry();
   while (currShape != NULL)
   {
      // Detach the PhysX shape from the actor
      rigid_actor->detachShape(*currShape->getShape());

      // Move onto the next shape
      currShape = (PhysXShape *) shapes->getNextEntry();
   }

   // Remove all the shapes from the map and delete the lists, which should
   // clean up the shapes as well
   actor_shapes->clear();
   delete shapeIds;
   delete shapes;

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


void PhysXRigidActor::setName(char * name)
{
   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Set the new name
   if (name != NULL)
      actor_name->setString(name);
   else
      actor_name->setString("");

   // Update PhysX actor with the new name
   rigid_actor->setName(actor_name->getString());

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


char * PhysXRigidActor::getName()
{
   char *   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Return the current name of this actor
   result = actor_name->getString();

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return the name of the actor
   return result;
}


float PhysXRigidActor::getMass()
{
   float   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Only dynamic actors can have a mass
   if (actor_type == DYNAMIC)
   {  
      result = ((PxRigidDynamic *) rigid_actor)->getMass();  
   }
   else
   {
      // Can't add force to non-dynamic actors
      result = 0.0f;
   }

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return the resulting mass
   return result;
}


void PhysXRigidActor::clearAllForces()
{
    // These methods check if the actor is dynamic so there is no reason to
    // check for that here, also zero out all motion on the PhysX object
    setLinearVelocity(0.0f, 0.0f, 0.0f);
    setAngularVelocity(0.0f, 0.0f, 0.0f);
}


bool PhysXRigidActor::addForce(PxVec3 force)
{
   bool   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Only dynamic actors should have forces applied
   if (actor_type == DYNAMIC)
   {
      ((PxRigidDynamic *) rigid_actor)->addForce(force);
      result = true;
   }
   else
   {
      // This is not a dynamic actor, so forces cannot be applied
      result = false;
   }

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return whether the force application was successful
   return result;
}


bool PhysXRigidActor::addTorque(PxVec3 torque)
{
   bool   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Only dynamic actors should have torque applied
   if (actor_type == DYNAMIC)
   {
      // Cast the actor into a dynamic actor and apply the torque
      ((PxRigidDynamic *) rigid_actor)->addTorque(torque);
      result = true;
   }
   else
   {
      // This is not a dynamic actor, so torque cannot be applied
      result = false;
   }

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return whether the torque application was successful
   return result;
}


void PhysXRigidActor::setTransformation(float posX, float posY, float posZ,
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

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Change the global position and orientation to the new transform
   rigid_actor->setGlobalPose(transform);

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


void PhysXRigidActor::setPosition(ActorPosition pos)
{
   PxTransform   transform;
   PxQuat        quaternion;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Create the new position with the given values
   quaternion = rigid_actor->getGlobalPose().q;
   transform = PxTransform(PxVec3(pos.x, pos.y, pos.z), quaternion);

   // Update the actor's position and indicate that it should be
   // woken up so that it can be updated during the simulation step
   rigid_actor->setGlobalPose(transform);

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


ActorPosition PhysXRigidActor::getPosition()
{
   PxVec3          position;
   ActorPosition   retPosition;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Acquire the current position of this actor
   position = rigid_actor->getGlobalPose().p;

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Store the position values so that PhysX does not change the values while
   // the information is being used
   retPosition.x = position.x;
   retPosition.y = position.y;
   retPosition.z = position.z;

   // Return the current position of this actor
   return retPosition;
}


void PhysXRigidActor::setRotation(ActorOrientation orient)
{
   PxTransform   transform;
   PxVec3        position;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Get the current position to prevent changes to the position of this actor
   // when the orientation changes
   position = rigid_actor->getGlobalPose().p;

   // Create a new transform with the updated orientation and original position
   transform = PxTransform(position,
                           PxQuat(orient.x, orient.y, orient.z, orient.w));

   // Update the actor to use the new orientation
   rigid_actor->setGlobalPose(transform);

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


ActorOrientation PhysXRigidActor::getRotation()
{
   PxQuat             quaternion;
   ActorOrientation   retOrientation;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Acquire the current orienation of this actor
   quaternion = rigid_actor->getGlobalPose().q;

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Store the orientation into a returnable value so that PhysX does not
   // change the values while the information is being used
   retOrientation.x = quaternion.x;
   retOrientation.y = quaternion.y;
   retOrientation.z = quaternion.z;
   retOrientation.w = quaternion.w;

   // Return the current orientation of this actor
   return retOrientation;
}


void PhysXRigidActor::setLinearVelocity(float x, float y, float z)
{
   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Update the actor's linear velocity
   if (actor_type == DYNAMIC)
      ((PxRigidDynamic *) rigid_actor)->setLinearVelocity(PxVec3(x, y, z));

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


void PhysXRigidActor::setAngularVelocity(float x, float y, float z)
{
   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Update the actor's angular velocity
   if (actor_type == DYNAMIC)
   {
      ((PxRigidDynamic *) rigid_actor)->setAngularVelocity(PxVec3(x, y, z));
   }

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


void PhysXRigidActor::setGravity(float x, float y, float z)
{
   // TODO: Currently commands for changing the gravity are not handled by
   // OpenSim except to turn gravity on and off.
}


void PhysXRigidActor::enableGravity(bool enabled)
{
   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

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

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);
}


bool PhysXRigidActor::isDynamic()
{
   bool   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Use the ActorType enumertation to determine if the actor is dynamic or
   // static
   if (actor_type == DYNAMIC)
      result = true;
   else
      result = false;

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return whether the actor is static or dynamic
   return result;
}


bool PhysXRigidActor::setMass(float mass)
{
   bool   result;

   // Ensure that the following operations on the actor are thread-safe
   pthread_mutex_lock(&actor_mutex);

   // Only dynamic actors can have mass
   if (actor_type == DYNAMIC)
   {
      result = PxRigidBodyExt::setMassAndUpdateInertia(
                *((PxRigidDynamic *) rigid_actor), mass);
   }
   else
   {
      // Not allowed to set mass on non-dynamic actors
      result = false;
   }

   // Now that the operations are complete, unlock the mutex
   pthread_mutex_unlock(&actor_mutex);

   // Return whether the operation was successful
   return result;
}


void PhysXRigidActor::updateDensity()
{
   PxReal *        densities;
   atList *        shapeList;
   PhysXShape *    currShape;
   int             shapeIndex;
   PxRigidBody *   rigidBody;

   // Check to see if this actor is static; if it is, densities don't
   // need to be updated as they don't matter
   if (actor_type == STATIC)
   {
      // Exit out, as the density doesn't matter
      return;
   }

   // Retrieve the shapes attached to this actor
   shapeList = new atList();
   actor_shapes->getSortedList(NULL, shapeList);

   // Check to see if the shape list is invalid
   if (shapeList == NULL)
   {
      // Exit out, since there are no shapes with which to calculate density
      return;
   }

   // Allocate an array that will hold the densities from the shapes; this array
   // will be passed into PhysX
   densities = new PxReal[shapeList->getNumEntries()];

   // Go through each of the shapes and store its density
   shapeIndex = 0;
   currShape = (PhysXShape *) shapeList->getFirstEntry();
   while (currShape != NULL)
   {
      // Check to see if the current shape's density is valid
      if (currShape->getDensity() <= 0.0f)
      {
         // Display a warning and use the default density, as an invalid
         // density will cause PhysX to crash
         densities[shapeIndex] = default_density;
      }
      else
      {
         // Record the density of the current shape
         densities[shapeIndex] = currShape->getDensity();
      }

      // Move onto the next shape
      currShape = (PhysXShape *) shapeList->getNextEntry();
      shapeIndex++;
   }

   // Update the PhysX actor's density
   rigidBody = (PxRigidBody *)rigid_actor;
   PxRigidBodyExt::updateMassAndInertia(*rigidBody, densities,
      shapeList->getNumEntries());

   // Clean up the temporary shape list and densities
   shapeList->removeAllEntries();
   delete shapeList;
   delete[] densities;
}

