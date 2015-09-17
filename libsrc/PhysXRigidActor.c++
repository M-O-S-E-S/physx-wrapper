#include "PhysXRigidActor.h++"


PhysXRigidActor::PhysXRigidActor(PxPhysics * physics, unsigned int id,
   float x, float y, float z, ActorType type)
{
   PxTransform   actorPos;

   // Create the position transform for this actor
   actorPos = PxTransform(PxVec3(x, y, z));

   // Determine if the actor is dynamic or static
   if (type == DYNAMIC)
   {
      // Create a dynamic actor for physical interactions
      dynamic_actor = physics->createRigidDynamic(actorPos);
      rigid_actor = (PxRigidActor *)dynamic_actor;

      // Enable continuous collision detection for this dynamic actor
      dynamic_actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
   }
   else
   {
      // Create a static actor for physical interactions
      static_actor = physics->createRigidStatic(actorPos);
      rigid_actor = (PxRigidActor *) static_actor;
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

   // Create the position transform for this actor
   actorPos = PxTransform(PxVec3(x, y, z), rot);

   // Determine the type of actor that needs to be created for this actor
   if (type == DYNAMIC)
   {
      // Create a dynamic actor for physical interactions
      dynamic_actor = physics->createRigidDynamic(actorPos);
      rigid_actor = (PxRigidActor *)dynamic_actor;

      // Enable continuous collision detection for this dynamic actor
      dynamic_actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
   }
   else
   {
      // Create a static actor for physical interactions
      static_actor = physics->createRigidStatic(actorPos);
      rigid_actor = (PxRigidActor *) static_actor;
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

PxRigidDynamic * PhysXRigidActor::getRigidDynamic()
{
   // Return the current PhysX dynamic actor
   return dynamic_actor;
}

PxRigidStatic * PhysXRigidActor::getRigidStatic()
{
   // Return the current PhysX static actor
   return static_actor;
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
   if (dynamic_actor != NULL)
      dynamic_actor->setLinearVelocity(PxVec3(x, y, z));
}


void PhysXRigidActor::setAngularVelocity(float x, float y, float z)
{
   // Update the actor's angular velocity
   if (dynamic_actor != NULL)
   {
      dynamic_actor->setAngularVelocity(PxVec3(x, y, z));
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
      rigid_actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
   else
      rigid_actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, false);
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

