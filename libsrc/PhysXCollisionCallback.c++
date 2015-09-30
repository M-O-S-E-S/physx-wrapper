
// PhysX Wrapper
//
// Copyright 2015 University of Central Florida

// This library wraps up the native calls to NVIDIA's PhysX API and
// provides higher-level functions to C# for use.

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


#include "PhysXCollisionCallback.h++"

#include "PxPhysicsAPI.h"
#include "atInt.h++"

using namespace physx;


CollisionProperties * PhysXCollisionCallback::getCollisions(uint * nbCollisions)
{
   // Set the number of collisions
   *nbCollisions = (uint) collision_count;

   // Reset the number of collisions that have occurred since they have now
   // been sent
   collision_count = 0;

   // And return the collision update
   return collision_array;
}


void PhysXCollisionCallback::setCollisionsArray(
   CollisionProperties * collisions, int max)
{
   // Save the pinned array for collision updates
   collision_array = collisions;
   
   // Store the max size of the collision array
   max_collisions = max;

   // Initialize the number of collisions to zero since no collisions have
   // occurred 
   collision_count = 0;
}


void PhysXCollisionCallback::onConstraintBreak(
   PxConstraintInfo * constraints, PxU32 count)
{
   // This is an override for the PxSimulationEventCallback and is currently
   // not needed
}


void PhysXCollisionCallback::onWake(PxActor ** actors, PxU32 count)
{
   // This is an override for the PxSimulationEventCallback and is currently
   // not needed
}


void PhysXCollisionCallback::onSleep(PxActor ** actors, PxU32 count)
{
   // This is an override for the PxSimulationEventCallback and is currently
   // not needed
}


void PhysXCollisionCallback::onContact(
   const PxContactPairHeader& pairHeader, const PxContactPair * pairs,
   PxU32 nbPairs)
{
   const PxContactPair *   contactPair;
   int                     bufferSize;
   int                     numContacts;
   atInt *                 actor1ID;
   atInt *                 actor2ID;
   PxContactPairPoint *    contactPoints;

   // TODO: why is the buffer size 64?
   // Initialize the buffer that will hold the contact points of collisions
   bufferSize = 64;
   contactPoints = new PxContactPairPoint[bufferSize];

   // Iterate through each of the contact pairs
   for (int i = 0; i < nbPairs; i++)
   {
      // Current contact pair
      contactPair = &pairs[i];

      // Check if contact has begun between the two actors
      if ((contactPair->events & PxPairFlag::eNOTIFY_TOUCH_FOUND) ||
          (contactPair->events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS))
      {
         // For the actors that are involved in the contact, get
         // their IDs which are stored in the user data
         actor1ID = reinterpret_cast<atInt *>(pairHeader.actors[0]->userData);
         actor2ID = reinterpret_cast<atInt *>(pairHeader.actors[1]->userData);

         // Check that both actors IDs have been successfully retrieved
         if (actor1ID != NULL && actor2ID != NULL)
         {
            // Get the number of contacts from the contact pair
            numContacts =
               contactPair->extractContacts(contactPoints, bufferSize);

            // Loop through all of the collisions that occurred during the last
            // time step in PhysX
            for (int j = 0; j < numContacts; j++)
            {
               // We are only able to make a certain amount of updates
               // for each simulation step
               if (collision_count >= max_collisions)
                  break;

               // Store the IDs of the actors involved in the collision
               collision_array[collision_count].ActorId1 = actor1ID->getValue();
               collision_array[collision_count].ActorId2 = actor2ID->getValue();

               // Get the position and normal of the contact and store
               // them in the array of updated contact collisions, so
               // they may be sent out
               collision_array[collision_count].Penetration =
                  contactPoints[j].separation;
               collision_array[collision_count].PositionX =
                  contactPoints[j].position.x;
               collision_array[collision_count].PositionY =
                  contactPoints[j].position.y;
               collision_array[collision_count].PositionZ =
                  contactPoints[j].position.z;
               collision_array[collision_count].NormalX =
                  contactPoints[j].normal.x;
               collision_array[collision_count].NormalY =
                  contactPoints[j].normal.y;
               collision_array[collision_count].NormalZ =
                  contactPoints[j].normal.z;

               // Increment count for total collisions
               collision_count++;
            }

            break;
         }
      }
   }
}


void PhysXCollisionCallback::onTrigger(PxTriggerPair * pairs, PxU32 count)
{
   // This is an override for the PxSimulationEventCallback and is currently
   // not needed
}

