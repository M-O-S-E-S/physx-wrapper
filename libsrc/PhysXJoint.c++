
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


#include "PhysXJoint.h++"


PhysXJoint::PhysXJoint(PxJoint * joint, unsigned int jointID,
   unsigned int actor1ID, unsigned int actor2ID)
{
   // Initialize members based on the given parameters
   px_joint = joint;
   joint_id = new atInt(jointID);
   actor1_id = new atInt(actor1ID);
   actor2_id = new atInt(actor2ID);
}


PhysXJoint::~PhysXJoint()
{
   // Clean up the underlying PhysX joint
   if (px_joint != NULL)
      px_joint->release();

   // Clean up the ID members
   if (joint_id != NULL)
      delete joint_id;
   if (actor1_id != NULL)
      delete actor1_id;
   if (actor2_id != NULL)
      delete actor2_id;
}


void PhysXJoint::setID(unsigned int id)
{
   // Delete the current ID, if one already exists
   if (joint_id != NULL)
      delete joint_id;

   // Update the unique identifier of the joint
   joint_id = new atInt(id);
}


atInt PhysXJoint::getID()
{
   // Return the unique identifier of the joint
   return *joint_id;
}


void PhysXJoint::setFirstActorID(unsigned int id)
{
   // Delete the current actor's ID, if it already exists
   if (actor1_id != NULL)
      delete actor1_id;

   // Update the unique identifier of the first actor to which the joint
   // is attached to
   actor1_id = new atInt(id);
}


atInt PhysXJoint::getFirstActorID()
{
   // Return the unique identifier of the first actor to which the joint
   // is attached to
   return *actor1_id;
}


void PhysXJoint::setSecondActorID(unsigned int id)
{
   // Delete the current actor's ID, if it already exists
   if (actor2_id != NULL)
      delete actor2_id;

   // Update the unique identifier of the second actor to which the joint
   // is attached to
   actor2_id = new atInt(id);
}


atInt PhysXJoint::getSecondActorID()
{
   // Return the unique identifier of the second actor to which the joint
   // is attached to
   return *actor2_id;
}


void PhysXJoint::setJoint(PxJoint * joint)
{
   // Remove the current existing joint
   if (px_joint != NULL)
      px_joint->release();

   // Update the underlying PhysX joint
   px_joint = joint;
}


PxJoint * PhysXJoint::getJoint()
{
   // Return the underlying PhysX joint held by this object
   return px_joint;
}

