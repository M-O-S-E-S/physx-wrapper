
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


#include "PxPhysicsAPI.h"

using namespace std;
using namespace physx;

/// Struct for storing collision data.
struct CollisionProperties
{
   unsigned int ActorId1;
   unsigned int ActorId2;
   float PositionX;
   float PositionY;
   float PositionZ;
   float NormalX;
   float NormalY;
   float NormalZ;
   float Penetration;
};


/// Child class of the PhysX PxSimulationEventCallback that is used to acquire
/// collision information for OpenSim PhysXPlugin

class PhysXCollisionCallback : public PxSimulationEventCallback
{
   private:
      /// The array that holds the collision information.
      CollisionProperties * collision_array;
        
      /// The number of collisions that have occurred during this simulation 
      /// step.
      int collision_count;

      /// The max number of collisions that can be handled. 
      /// The limiting factor is the size of the array that was pinned to 
      /// memory, that transfers the collision data.
      int max_collisions;

   public:
      /// Method to acquire the collisions that occurred during the PhysX 
      /// simulation.
      ///
      /// @param nbCollisions This pointer is being passed by reference in 
      /// order to save the number of collisions that will be returned.
      /// 
      /// @return An array of collision properties that hold the information
      /// about the collisions that occurred this frame.
      ///
      CollisionProperties * getCollisions(uint * nbCollisions);

      /// Method to store the array of collisions for the collision callback 
      /// class.
      ///
      /// @param collisions The array of collisions that has been pinned to 
      /// memory for information transfer between managed and unmanaged code.
      /// @param max The maximum number of collisions that the array can hold.
      ///
      void setCollisionsArray(CollisionProperties * collisions, int max);

      /// Inherited method that is currently not being used.
      ///
      void onConstraintBreak(PxConstraintInfo * constraints, PxU32 count);

      /// Inherited method that is currently not being used.
      ///
      void onWake(PxActor ** actors, PxU32 count);
      
      /// Inherited method that is currently not being used.
      ///
      void onSleep(PxActor ** actors, PxU32 count);
      
      /// Inherited method that notifies us when contact events occur for 
      /// actors that have requested reporting.
      ///
      /// @param pairHeader Information on the two actors whose shapes 
      /// triggered a contact report.
      /// @param pairs The contact pairs of two actors for which contact 
      /// reports have been requested.
      /// @param nbPairs The number of provided contact pairs.
      ///
      void onContact(const PxContactPairHeader& pairHeader, 
         const PxContactPair * pairs, PxU32 nbPairs);
      
      /// Inherited method that is currently not being used.
      ///
      void onTrigger(PxTriggerPair *, PxU32);
};
