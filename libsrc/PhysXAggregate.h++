
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

#ifndef PHYSX_AGGREGATE_H
#define PHYSX_AGGREGATE_H


#include "PxPhysicsAPI.h"
#include "PhysXRigidActor.h++"

#include "atInt.h++"
#include "atItem.h++"

using namespace physx;

class PhysXAggregate : public atItem
{
    public:
        /// Constructor.
        ///
        /// @param aggregate The underlying PhysX PxAggregate that will be held
        /// by this PhysXAggregate object.
        /// @param aggregateId The unique identifier for the PxAggregate.
        ///
        PhysXAggregate(PxAggregate * aggregate, unsigned int aggregateId);

        /// Deconstructor.
        ///
        ~PhysXAggregate();

        /// Updates the unique identifier of the PhysX PxAggregate.
        ///
        /// @param id The new unique identifier of the PhysX PxAggregate.
        ///
        void   setID(unsigned int id);

        /// Returns the unique identifier of the PhysX PxAggregate.
        ///
        /// @return The unique identifier of the PhysX PxAggregate.
        ///
        atInt *   getID();
            
        /// Updates the underlying PhysX PxAggregate held by this object.
        ///
        /// @param aggregate The new underlying PhysX PxAggregate.
        ///
        void   setAggregate(PxAggregate * aggregate);

        /// Returns the underlying PhysX PxAggregate held by this object.
        ///
        /// @return The underlying PhysX PxAggregate held by this object.
        ///
        PxAggregate *   getAggregate();

        /// Adds the given PhysX RigidActor to the PxAggregate.
        ///
        /// @param actor The PhysXRigidActor to be added
        ///
        /// @retrun bool representing if the actor was properly added
        ///
        bool   addActor(PhysXRigidActor * actor);

        /// Removes the given PhysX RigidActor from the PxAggregate.
        ///
        /// @param actor The PhysXRigidActor to be added
        ///
        /// @retrun bool representing if the actor was properly removed
        ///
        bool   removeActor(PhysXRigidActor * actor);

        /// This method determines whether or not the PhysXAggregate object
        /// is equal to the object that is passed as an argument.
        ///
        /// @param otherItem The other PhysXRigidActor atItem to be equated
        /// 
        /// @return bool representing if the two atItems are equal or not
        ///
        bool   equals(atItem * otherItem);

        /// This method compares this PhysXAggregate object against the given
        /// atItem argument.
        ///
        /// @param otherItem The other PhysXAggregate item to be compared to
        ///
        /// @return int representing the comparison on the two atItems
        ///
        int   compare(atItem * otherItem);

        /// Removes and releases any memory used by this object.
        ///
        void   release();
    protected:
        /// The unique identifier of the PhysX PxAggregate.
        atInt * aggregate_id;

        /// The underlying PhysX PxAggregate being held by this object.
        PxAggregate * px_aggregate;
};

#endif