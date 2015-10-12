
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

#include "PhysXAggregate.h++"


PhysXAggregate::PhysXAggregate(PxAggregate * aggregate, unsigned int aggregateId)
{
    px_aggregate = aggregate;
    aggregate_id = new atInt(aggregateId);
}


PhysXAggregate::~PhysXAggregate()
{
    // Clean up, and release the underlying PhysX aggregate
    if (px_aggregate != NULL)
    {
        px_aggregate->release();
    }

    delete aggregate_id;
}


void PhysXAggregate::setID(unsigned int id)
{
    // Delete the current ID, if one already exists
    if (aggregate_id != NULL)
    {
        delete aggregate_id;
    }

    // Update the unique identifier of the aggregate
    aggregate_id = new atInt(id);
}


atInt * PhysXAggregate::getID()
{
    // Return the unique identifier of the aggregate
    return aggregate_id;
}


void PhysXAggregate::setAggregate(PxAggregate * aggregate)
{
    // Remove the current existing aggregate
    if (px_aggregate != NULL)
    {
        px_aggregate->release();
    }

    // Update the underlying PhysX aggregate
    px_aggregate = aggregate;
}


PxAggregate * PhysXAggregate::getAggregate()
{
    // Return the underlying PhysX aggregate held by this object
    return px_aggregate;
}


bool PhysXAggregate::addActor(PhysXRigidActor * actor)
{
    // Return the result of adding the actor to the
    // PhysX PxAggregate object
    return px_aggregate->addActor(*actor->getActor());
}


bool PhysXAggregate::removeActor(PhysXRigidActor * actor)
{
    // Return the result of removing the actor from the
    // PhysX PxAggregate object
    return px_aggregate->removeActor(*actor->getActor());
}


void PhysXAggregate::release()
{
    // Release the PhysX PxAggregate class from memory
    px_aggregate->release();

    // Remove and delete the aggregate identifier from memory
    delete aggregate_id;
}


bool PhysXAggregate::equals(atItem * otherItem)
{
    PhysXAggregate * aggregateItem;

    // Try to cast the other item to an instance of PhysXAggregate
    aggregateItem = dynamic_cast<PhysXAggregate *>(otherItem);

    // Check to see if the other item is valid
    if(aggregateItem != NULL)
    {
        // Return the int comparison between the two identifiers
        return this->getID()->getValue() == aggregateItem->getID()->getValue();
    }

    // Otherwise, the types of item's didn't match so return false
    return false;
}


int PhysXAggregate::compare(atItem * otherItem)
{
    PhysXAggregate * aggregateItem;

    // Try to cast the other item to an instance of PhysXAggregate
    aggregateItem = dynamic_cast<PhysXAggregate *>(otherItem);

    // Check to see if the other item is valid
    if(aggregateItem != NULL)
    {
        // Return the int difference between the two identifiers
        return this->getID()->getValue() - aggregateItem->getID()->getValue();
    }

    // Otherwise, return the default atItem comparison
    return atItem::compare(otherItem);
}