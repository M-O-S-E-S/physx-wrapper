
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


/// This is a header file for the PhysXLib.c++ class. It will not actually be
/// included inside of any file and is used for doxygen commenting. It is also
/// the main class of the PhysXWrapper and used as the bridge to link c# with
/// c++.

/// The PhysX foundation that will be used throughout the PhysXWrapper.
static PxFoundation *   px_foundation;

/// The PxPhysics object that will be used throughout the PhysXWrapper.
static PxPhysics *   px_physics;

/// The PxScene that will hold the entire physical scene.
static PxScene *   px_scene;

/// The PxCooking object that is used when creating the height field.
static PxCooking *   px_cooking;

/// The default error callback of PhysX used when creating the foundation.
static PxDefaultErrorCallback   error_callback;

/// The default allocator of PhysX used when creating the foundation.
static PxDefaultAllocator   allocator_callback;

/// Object that will handle the receiving and storage of all collisions within
/// the physical scene.
static PhysXCollisionCallback *   px_collisions;

/// Rigid static physical object that will hold the ground plane of the
/// physical scene.
static PxRigidStatic *   ground_plane;

/// Flag that tracks when the scene has been initialized.
static int   scene_initialized;

/// The max number of updates that can be passed from PhysXWrapper.
/// This number is based on the size of the pinned memory array responsible for
/// passing updates from the PhysXWrapper.
static int   max_updates;

/// Map of all the actors in the current scene with keys being equal to the
/// actor's id stored inside an atInt and the value being the actor.
static atMap *   actor_map;

/// Map of all the joints in the current scene with keys being equal to
// the joint's id stored in an atInt and the value being the joint itself.
static atMap *   joint_map;

/// PvdConnection used to establish a stream with PhysX visual debugger.
static debugger::comm::PvdConnection *   theConnection;

/// A global value to store a specific ID for the terrain since none will be
/// provided.
static atInt *   terrain_id;

/// The height field scale that will be used to adjust the height field to an
/// integer from a float and vice a versa.
static float   height_field_scale;

/// Array of updates that is shared.
static EntityProperties *   update_array;

/// Array of collisions that is shared.
static CollisionProperties *   collisions_array;

/// A struct shared in order to pass data consistently through the update array.
struct EntityProperties;


/// Method to create an actor either dynamic or static with given id, name, and
/// position.
///
/// @param id Id of the physical actor being created.
/// @param name Name of the physical actor being created.
/// @param x The x value of the position of the physical actor in the scene.
/// @param y The y value of the position of the physical actor in the scene.
/// @param z The z value of the position of the physical actor in the scene.
/// @param isDynamic Flag that will control whether the physical actor is a
/// RigidStatic or RigidDynamic.
///
/// @return The PhysXRigidActor that represents the physical object.
///
PhysXRigidActor *   createRigidActor(unsigned int id, char * name, float x,
   float y, float z, bool isDynamic);

/// Method to create an actor either dynamic or static with given id, name, and
/// position.
///
/// @param id Id of the physical actor being created.
/// @param name Name of the physical actor being created.
/// @param x The x value of the position of the physical actor in the scene.
/// @param y The y value of the position of the physical actor in the scene.
/// @param z The z value of the position of the physical actor in the scene.
/// @param Rot The orientation of the physical object in the scene as a
/// quaternion.
/// @param isDynamic Flag that will control whether the physical actor is a
/// RigidStatic or RigidDynamic.
///
/// @return The PhysXRigidActor that represents the physical object.
///
PhysXRigidActor *   createRigidActor(unsigned int id, char * name, float x,
   float y, float z, PxQuat Rot, bool isDynamic);

/// Method to fetch the actor from the map of actors.
///
/// @param id The id of the actor that is being fetched.
///
/// @return The actor with the given id or null if the actor was not inside of
/// the map of actors.
///
PhysXRigidActor *   getActor(unsigned int id);

/// Method to fetch the actor from the map of actors.
///
/// @param id The id of the actor that is being fetched.
///
/// @return The actor with the given id or null if the actor was not inside of
/// the map of actors.
///
PhysXRigidActor *   getActor(atInt * id);

/// Custom filter shader used for collision filtering and to customize the
/// collection of flags describing the actions to take on a collision pair.
///
/// TODO: I don't know the parameters so either Chandler or Rob will have to
/// make a pass over this and add in the parameters and return value.
///
PxFilterFlags   contactFilterShader(PxFilterObjectAttributes attributes0,
   PxFilterData filterData0, PxFilterObjectAttributes attributes1,
   PxFilterData filterData1, PxPairFlags& pairFlags,
   const void * constantBlock, PxU32 constantBlockSize);

/// Method to connect the PhysXWrapper to a running PhysX Visual Debugger.
///
void   startVisualDebugger();


extern "C"
{
   /// Initializes the foundation, physics, cooking, collisions, and visual
   /// debugger.
   ///
   /// @return 1 if successfully initialized and 0 otherwise.
   ///
   int   initialize();

   /// Cleans up the PhysXWrapper by releasing the visual debugger, physics,
   /// and foundation in that order.
   ///
   void   release();

   /// Initialize the update array for updating the physical object properties
   /// after every simulate call.
   ///
   /// @param updateArray The array that has been pinned to memory and will be
   /// transfering the updates from the unmanaged code to managed code.
   /// @param maxUpdates The size of the updateArray which in turn determines
   /// how many updates can be sent after each simulate call.
   ///
   void   initEntityUpdate(EntityProperties * updateArray, int maxUpdates);

   /// Initialize the collision array for updating the physical object
   /// collisions after every simulate call.
   ///
   /// @param collisionArray The array that has been pinned to memory and will
   /// be transferring the collisions from the unmanaged code to managed code.
   /// @param maxUpdates The size of the collisionArray which in turn determines
   /// how many collisions can be sent after each simulate call.
   ///
   void   initCollisionUpdate(CollisionProperties * collisionArray,
      int maxUpdates);

   /// Create the scene for the physical objects and determine what hardware is
   /// running PhysX.
   ///
   /// @param gpuEnabled Flag that tells the method to set up the GPU for
   /// PhysX.
   /// @param cpuEnabled Flag that tells the method to set up the CPU for
   /// PhysX.
   /// @param cpuMaxThreads Number of threads that the CPU should use for
   /// PhysX.
   ///
   /// @return 1 if the scene is set up correctly and 0 if the scene was
   /// unable to be created.
   ///
   void   createScene(bool gpuEnabled, bool cpuEnabled, int cpuMaxThreads);

   /// Call the scene release to clean up the scene.
   ///
   void   releaseScene();

   /// Method to create a PxAggregate to store scene actors in.
   ///
   /// @param id The id of the aggregate to be created.
   ///
   void   createAggregate(unsigned int id);

   /// Method to remove a PxAggregate that stores scene actors.
   ///
   /// @param id The id of the aggregate to be destroyed.
   ///
   void   removeAggregate(unsigned int id);

   /// Method to add a PxActor to a PxAggregate by their ids.
   ///
   /// @param aggregateId The id of the aggregate that the actor
   /// will be added to.
   /// @param actorId The id of the actor to be added to the specified
   /// aggregate instance.
   ///
   bool   addToAggregate(unsigned int aggregateId, unsigned int actorId);

   /// Method to remove a PxActor from a PxAggregate by their ids.
   ///
   /// @param aggregateId The id of the aggregate that the actor
   /// will be removed from.
   /// @param actorId The id of the actor to be removed in the specified
   /// aggregate instance.
   ///
   bool   removeFromAggregate(unsigned aggregateId, unsigned int, actorId);

   /// Method to create an actor either dynamic or static with given id, name,
   /// and position.
   ///
   /// @param id Id of the physical actor being created.
   /// @param name Name of the physical actor being created.
   /// @param x The x value of the position of the physical actor in the scene.
   /// @param y The y value of the position of the physical actor in the scene.
   /// @param z The z value of the position of the physical actor in the scene.
   /// @param isDynamic Flag that will control whether the physical actor is a
   /// RigidStatic or RigidDynamic.
   ///
   void   createActor(unsigned int id, char * name, float x, float y, float z,
      bool isDynamic);

   /// Method to attach a sphere shape to an existing actor.
   ///
   /// @param id Unique identifier of the actor to which the shape is being 
   /// attached.
   /// @param shapeId Unique identifier of the shape being attached.
   /// @param staticFriction The static friction to be used for this sphere
   /// when sliding against other objects of the scene.
   /// @param dynamicFriction The dynamic friction to be used for this sphere
   /// when sliding against other objects of the scene.
   /// @param restitution The bounciness of this shape when colliding with other
   /// objects of the scene.
   /// @param radius The radius of the sphere being attached.
   /// @param x The position of the sphere along the x-axis relative to the
   /// position of the actor to which the sphere is being attached.
   /// @param y The position of the sphere along the y-axis relative to the
   /// position of the actor to which the sphere is being attached.
   /// @param z The position of the sphere along the z-axis relative to the
   /// position of the actor to which the sphere is being attached.
   /// @param density The density of the sphere.
   ///
   void   attachSphere(unsigned int id, unsigned int shapeId,
                       float staticFriction, float dynamicFriction,
                       float restitution, float radius, float x, float y,
                       float z, float density);

   /// Method to attach a box shape to an existing actor.
   ///
   /// @param id Unique identifier of the actor to which the shape is being
   /// attached.
   /// @param shapeId Unique identifier of the shape being attached.
   /// @param staticFriction The static friction to be used for this box
   /// when sliding against other objects of the scene.
   /// @param dynamicFriction The dynamic friction to be used for this box
   /// when sliding against other objects of the scene.
   /// @param restitution The bounciness of this shape when colliding with other
   /// objects of the scene.
   /// @param halfX Half the length of the box.
   /// @param halfY Half the width of the box.
   /// @param halfZ Half the height of the box.
   /// @param x The position of the box along the x-axis relative to the
   /// position of the actor to which the box is being attached.
   /// @param y The position of the box along the y-axis relative to the
   /// position of the actor to which the box is being attached.
   /// @param z The position of the box along the z-axis relative to the
   /// position of the actor to which the box is being attached.
   /// @param rotX The x-value of the quaternion representing orientation of the
   /// box relative to the actor.
   /// @param rotY The y-value of the quaternion representing orientation of the
   /// box relative to the actor.
   /// @param rotZ The z-value of the quaternion representing orientation of the
   /// box relative to the actor.
   /// @param rotW The w-value of the quaternion representing orientation of the
   /// box relative to the actor.
   /// @param density The density of the box.
   ///
   void   attachBox(unsigned int id, unsigned int shapeId, float staticFriction,
                    float dynamicFriction, float restitution, float halfX,
                    float halfY, float halfZ, float x, float y, float z,
                    float rotX, float rotY, float rotZ, float rotW,
                    float density);

   /// Method to attach a capsule shape to an existing actor.
   ///
   /// @param id Unique identifier of the actor to which the shape is being
   /// attached.
   /// @param shapeId Unique identifier of the shape being attached.
   /// @param staticFriction The static friction to be used for this capsule
   /// when sliding against other objects of the scene.
   /// @param dynamicFriction The dynamic friction to be used for this capsule
   /// when sliding against other objects of the scene.
   /// @param restitution The bounciness of this shape when colliding with other
   /// objects of the scene.
   /// @param halfHeight Half the height of the capsule.
   /// @param radius The radius of the capsule.
   /// @param x The position of the capsule along the x-axis relative to the
   /// position of the actor to which the capsule is being attached.
   /// @param y The position of the capsule along the y-axis relative to the
   /// position of the actor to which the capsule is being attached.
   /// @param z The position of the capsule along the z-axis relative to the
   /// position of the actor to which the capsule is being attached.
   /// @param rotX The x-value of the quaternion representing the orientation
   /// of the capsule relative to the actor.
   /// @param rotY The y-value of the quaternion representing the orientation
   /// of the capsule relative to the actor.
   /// @param rotZ The z-value of the quaternion representing the orientation
   /// of the capsule relative to the actor.
   /// @param rotW The w-value of the quaternion representing the orientation
   /// of the capsule relative to the actor.
   /// @param density The density of the capsule.
   ///
   void   attachCapsule(unsigned int id, unsigned int shapeId,
                        float staticFriction, float dynamicFriction,
                        float restitution, float halfHeight, float radius,
                        float x, float y, float z, float rotX, float rotY,
                        float rotZ, float rotW, float density);

   /// Method to attach a triangle mesh shape to an existing actor.
   ///
   /// @param id Unique identifier of the actor to which the shape is being
   /// attached.
   /// @param shapeId Unique identifier of the shape being attached.
   /// @param staticFriction The static friction to be used for this mesh
   /// when sliding against other objects of the scene.
   /// @param dynamicFriction The dynamic friction to be used for this mesh
   /// when sliding against other objects of the scene.
   /// @param restitution The bounciness of this shape when colliding with other
   /// objects of the scene.
   /// @param vertices The list of vertices that make up the triangle mesh.
   /// @param indices The list of indices that define the triangles in the mesh.
   /// @param vertexCount The number of vertices given for this mesh.
   /// @param indexCount The number of indices given for this mesh.
   /// @param x The position of the mesh along the x-axis relative to the
   /// position of the actor to which the mesh is being attached.
   /// @param y The position of the mesh along the y-axis relative to the
   /// position of the actor to which the mesh is being attached.
   /// @param z The position of the mesh along the z-axis relative to the
   /// position of the actor to which the mesh is being attached.
   /// @param rotX The x-value of the quaternion representing the orientation
   /// of the mesh relative to the actor.
   /// @param rotY The y-value of the quaternion representing the orientation
   /// of the mesh relative to the actor.
   /// @param rotZ The z-value of the quaternion representing the orientation
   /// of the mesh relative to the actor.
   /// @param rotW The w-value of the quaternion representing the orientation
   /// of the mesh relative to the actor.
   ///
   /// @remarks Triangle Meshes can only be attached to dynamic
   /// actors, so this method will fail if the given ID belongs to a static
   /// actor.
   ///
   void   attachTriangleMesh(unsigned int id, unsigned int shapeId,
                             float staticFriction, float dynamicFriction,
                             float restitution, float * vertices,
                             float * indices, int vertexCount, int indexCount,
                             float x, float y, float z, float rotX,
                             float rotY, float rotZ, float rotW);

   /// Method to attach a convex mesh to an existing actor.
   ///
   /// @param id Unique identifier of the actor to which the shape is being
   /// attached.
   /// @param shapeId Unique identifier of the shape being attached.
   /// @param staticFriction The static friction to be used for this mesh
   /// when sliding against other objects of the scene.
   /// @param dynamicFriction The dynamic friction to be used for this mesh
   /// when sliding against other objects of the scene.
   /// @param restitution The bounciness of this shape when colliding with other
   /// objects of hte scene
   /// @param vertices The list of vertices that make up the convex mesh.
   /// @param vertexCount The size of the list of vertices.
   /// @param x The position of the mesh along the x-axis relative to the
   /// position of the actor to which the mesh is being attached.
   /// @param y The position of the mesh along the y-axis relative to the
   /// position of the actor to which the mesh is being attached.
   /// @param z The position of the mesh along the z-axis relative to the
   /// position of the actor to which the mesh is being attached.
   /// @param rotX The x-value of the quaternion representing the orientation
   /// of the mesh relative to the actor.
   /// @param rotY The y-value of the quaternion representing the orientation
   /// of the mesh relative to the actor.
   /// @param rotZ The z-value of the quaternion representing the orientation
   /// of the mesh relative to the actor.
   /// @param rotW The w-value of the quaternion representing the orientation
   /// of the mesh relative to the actor.
   /// @param density The density of the convex mesh.
   ///
   void   attachConvexMesh(unsigned int id, unsigned int shapeId,
                           float staticFriction, float dynamicFriction,
                           float restitution, float * vertices, int vertexCount,
                           float x, float y, float z, float rotX, float rotY,
                           float rotZ, float rotW, float density);

   /// Method to remove and delete a shape attached to an actor.
   ///
   /// @param id The unique identifier of the actor from which the shape
   /// should be deleted.
   /// @param shapeId The unique identifier of the shape to be removed &
   /// deleted.
   ///
   void   removeShape(unsigned int id, unsigned int shapeId);

   /// Method to create a sphere actor in the physical scene.
   ///
   /// @param id The id of the actor.
   /// @param name The string name associated with this actor.
   /// @param x The x value of the position of this actor in the physical
   /// scene.
   /// @param y The y value of the position of this actor in the physical
   /// scene.
   /// @param z The z value of the position of this actor in the physical
   /// scene.
   /// @param shapeId The unique identifier of the sphere shape being attached
   /// to the actor.
   /// @param staticFriction The static friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param dynamicFriction The dynamic friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param restitution How bouncy this actor is when colliding with other
   /// actors in the physical scene.
   /// @param radius The radius of the sphere for the physical actor.
   /// @param density The density of the sphere.
   /// @param isDynamic Flag that determines if this is a static or dynamic
   /// actor.
   ///
   void   createActorSphere(unsigned int id, char * name, float x, float y, 
                            float z, unsigned int shapeId, float staticFriction,
                            float dynamicFriction, float restitution,
                            float radius, float density, bool isDynamic);

   /// Method to create a box actor in the physical scene.
   ///
   /// @param id The id of the actor.
   /// @param name The string name associated with this actor.
   /// @param x The x value of the position of this actor in the physical
   /// scene.
   /// @param y The y value of the position of this actor in the physical
   /// scene.
   /// @param z The z value of the position of this actor in the physical
   /// scene.
   /// @param shapeId The unique identifier of the box shape being attached to
   /// the actor.
   /// @param staticFriction The static friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param dynamicFriction The dynamic friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param restitution How bouncy this actor is when colliding with other
   /// actors in the physical scene.
   /// @param halfX Half the length of the box actor.
   /// @param halfY Half the height of the box actor.
   /// @param halfZ Half the width of the box actor.
   /// @param density The density of the box.
   /// @param isDynamic Flag that determines if this is a static or dynamic
   /// actor.
   ///
   void   createActorBox(unsigned int id, char * name, float x, float y, 
                         float z, unsigned int shapeId, float staticFriction,
                         float dynamicFriction, float restitution, float halfX,
                         float halfY, float halfZ, float density,
                         bool isDynamic);

   /// Method to create a capsule actor in the physical scene.
   ///
   /// @param id The id of the actor.
   /// @param name The string name associated with this actor.
   /// @param x The x value of the position of this actor in the physical
   /// scene.
   /// @param y The y value of the position of this actor in the physical
   /// scene.
   /// @param z The z value of the position of this actor in the physical
   /// scene.
   /// @param shapeId The unique identifier of the capsule shape being attached
   /// to the actor.
   /// @param rotX The x value of the quaternion representing the orientation 
   /// of the capsule shape relative to the actor.
   /// @param rotY The y value of the quaternion representing the orientation 
   /// of the capsule shape relative to the actor.
   /// @param rotZ The z value of the quaternion representing the orientation 
   /// of the capsule shape relative to the actor.
   /// @param rotW The w value of the quaternion representing the orientation 
   /// of the capsule shape relative to the actor.
   /// @param staticFriction The static friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param dynamicFriction The dynamic friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param restitution How bouncy this actor is when colliding with other
   /// actors in the physical scene.
   /// @param halfHeight Half the height of the capsule actor.
   /// @param radius The radius of the capsule for the physical actor.
   /// @param density The density of the capsule.
   /// @param isDynamic Flag that determines if this is a static or dynamic
   /// actor.
   ///
   void   createActorCapsule(unsigned int id, char * name, float x, float y, 
                             float z, unsigned int shapeId, float rotX,
                             float rotY, float rotZ, float rotW,
                             float staticFriction, float dynamicFriction,
                             float restitution, float halfHeight, float radius,
                             float density, bool isDynamic);

   /// Method to create a triangle mesh actor in the physical scene.
   ///
   /// @param id The id of the actor.
   /// @param name The string name associated with this actor.
   /// @param x The x value of the position of this actor in the physical
   /// scene.
   /// @param y The y value of the position of this actor in the physical
   /// scene.
   /// @param z The z value of the position of this actor in the physical
   /// scene.
   /// @param shapeId The unique identifier of the mesh being attached to
   /// actor.
   /// @param staticFriction The static friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param dynamicFriction The dynamic friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param restitution How bouncy this actor is when colliding with other
   /// actors in the physical scene.
   /// @param vertices The list of vertices of the triangles that make up the
   /// triangle mesh.
   /// @param indices The list of indices for the triangle mesh.
   /// @param vertexCount The size of the list of vertices.
   /// @param indexCount The size of the list of indices.
   /// @param isDynamic Flag that determines if this is a static or dynamic
   /// actor.
   ///
   void   createActorTriangleMesh(unsigned int id, char * name, float x,
                                  float y, float z, unsigned int shapeId,
                                  float staticFriction, float dynamicFriction,
                                  float restitution, float * vertices,
                                  float * indices, int vertexCount,
                                  int indexCount, bool isDynamic);

   /// Method to create a convex mesh actor in the physical scene.
   ///
   /// @param id The id of the actor.
   /// @param name The string name associated with this actor.
   /// @param x The x value of the position of this actor in the physical
   /// scene.
   /// @param y The y value of the position of this actor in the physical
   /// scene.
   /// @param z The z value of the position of this actor in the physical
   /// scene.
   /// @param shapeId The unique identifier of the mesh being attached to the
   /// actor.
   /// @param staticFriction The static friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param dynamicFriction The dynamic friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param restitution How bouncy this actor is when colliding with other
   /// actors in the physical scene.
   /// @param vertices The list of vertices of the mesh that make up the
   /// convex mesh.
   /// @param vertexCount The size of the list of vertices.
   /// @param density The density of the mesh.
   /// @param isDynamic Flag that determines if this is a static or dynamic
   /// actor.
   ///
   void   createActorConvexMesh(unsigned int id, char * name, float x, float y, 
                                float z, unsigned int shapeId,
                                float staticFriction, float dynamicFriction,
                                float restitution, float * vertices,
                                int vertexCount, float density, bool isDynamic);

   /// Remove an actor from the physical scene.
   ///
   /// @param id The id of the actor to be removed.
   ///
   void   removeActor(unsigned int id);

   /// Updates various physical properties of a shape.
   ///
   /// @param id The unique identifier of the actor to which the shape is
   /// attached.
   /// @param shapeId The unique identifier of the shape whose material
   /// properties are being modified.
   /// @param staticFriction The new static friction that will be used
   /// for the shape when sliding against other objects in the scene.
   /// @param dynamicFriction the new dynamic friction that will be used
   /// for the shape when sliding against other objects in the scene.
   /// @param restitution The bounciness of this shape.
   ///
   void   updateMaterialProperties(unsigned int id, unsigned int shapeId,
                                   float staticFriction, float dynamicFriction,
                                   float restitution);

   /// Get the mass of a physical actor.
   ///
   /// @param id The id of the actor for which the mass
   /// should be returned.
   ///
   /// @return The mass of the object
   ///
   float   getActorMass(unsigned int id);

   /// Apply torque to a physical actor.
   ///
   /// @param id The unique identifier of the actor to which the torque will
   /// be applied.
   /// @param torqueX The x-component of the torque being applied.
   /// @param torqueY The y-component of the torque being applied.
   /// @param torqueZ The z-component of the torque being applied.
   ///
   void   addTorque(unsigned int id, float torqueX, float torqueY,
                    float torqueZ);

   /// Updates an actors position and orientation inside the physical scene.
   ///
   /// @param id The id of the actor to be updated.
   /// @param posX The x value of the new physical translation in the scene.
   /// @param posY The y value of the new physical translation in the scene.
   /// @param posZ The z value of the new physical translation in the scene.
   /// @param rotX The x value of the quaternion representing the new rotation
   /// of the actor in the scene.
   /// @param rotY The y value of the quaternion representing the new rotation
   /// of the actor in the scene.
   /// @param rotZ The z value of the quaternion representing the new rotation
   /// of the actor in the scene.
   /// @param rotW The w value of the quaternion representing the new rotation
   /// of the actor in the scene.
   ///
   void   setTranslation(unsigned int id, float posX, float posY, float posZ,
      float rotX, float rotY, float rotZ, float rotW);

   /// Updates an actors position inside the physical scene.
   ///
   /// @param id The id of the actor to be updated.
   /// @param pos The position of the new physical translation in the scene.
   ///
   void   setPosition(unsigned int id, ActorPosition pos);

   /// Method to fetch the current position of an actor in the physical scene.
   ///
   /// @param id The id of the actor to be fetched.
   ///
   /// @return A struct of floats with the x, y, z values for the position of
   /// the actor in the physical scene.
   ///
   ActorPosition   getPosition(unsigned int id);

   /// Updates an actors rotation inside the physical scene.
   ///
   /// @param id The id of the actor to be updated.
   /// @param orient The quaternion representing the new rotation of
   /// the actor in the physical scene.
   ///
   void   setRotation(unsigned int id, ActorOrientation orient);

   /// Method to fetch the current orientation of an actor in the physical
   /// scene.
   ///
   /// @param id The id of the actor to be fetched.
   ///
   /// @return A struct of floats with the x, y, z, w values for the quaternion
   /// representing the orientation of the actor in the physical scene.
   ///
   ActorOrientation   getRotation(unsigned int id);

   /// Updates an actor with a new linear velocity.
   ///
   /// @param id The id of the actor to be updated.
   /// @param x The velocity of the actor in the x direction.
   /// @param y The velocity of the actor in the y direction.
   /// @param z The velocity of the actor in the z direction.
   ///
   void   setLinearVelocity(unsigned int id, float x, float y, float z);

   /// Updates an actor with a new angular velocity.
   ///
   /// @param id The id of the actor to be updated.
   /// @param x The velocity of the actor in the x direction.
   /// @param y The velocity of the actor in the y direction.
   /// @param z The velocity of the actor in the z direction.
   ///
   void   setAngularVelocity(unsigned int id, float x, float y, float z);

   /// Updates the scene gravity on an actor to the new values.
   ///
   /// @param id The id of the actor to be updated.
   /// @param x The amount of gravity applied to the actor in the x direction.
   /// @param y The amount of gravity applied to the actor in the y direction.
   /// @param z The amount of gravity applied to the actor in the z direction.
   ///
   void   setGravity(unsigned int id, float x, float y, float z);

   /// Enable or disable gravity on an actor.
   ///
   /// @param id The id of the actor that gravity is being changed.
   /// @param enabled Flag that determines if gravity is being enabled or
   /// disabled for this particular actor.
   ///
   void   enableGravity(unsigned int id, bool enabled);

   /// Updates the density of given shape attached to a given actor.
   ///
   /// @param id The unique identifier of the actor to which the desired shape
   /// is attached.
   /// @param shapeID The unique identifier of the desired shape.
   /// @param density The new density of the shape.
   ///
   void   updateShapeDensity(unsigned int id, unsigned int shapeID,
                             float density);

   /// Update the mass of a physical actor.
   ///
   /// @param id The id of the actor to be updated.
   /// @param mass The updated density of the actor.
   ///
   /// @return If the mass was updated
   ///
   bool   updateActorMass(unsigned int id, float mass);

   /// Method to create a ground plane to prevent actors from falling forever.
   ///
   /// @param x The x value of the position where the plane should be created.
   /// @param y The y value of the position where the plane should be created.
   /// @param z The z value of the position where the plane should be created.
   ///
   void   createGroundPlane(float x, float y, float z);

   /// Release the resources currently be used by the ground plane.
   ///
   void   releaseGroundPlane();

   /// Add a new terrain height map actor to the scene. This will delete the
   /// old terrain height map and replace it with the new one.
   ///
   /// @param terrainShapeID Currently not used by the program, but it should
   /// be the id associated with the terrain actor.
   /// @param regionSizeX The total length of the region.
   /// @param regionSizeY The total width of the region.
   /// @param rowSpacing The distance between height point rows inside of the
   /// posts array.
   /// @param columnSpacing The distance between height point columns inside of
   /// the posts array.
   /// @param posts The array of height values that will be used to generate
   /// the height field.
   ///
   void   setHeightField(unsigned int terrainActorID,
                         unsigned int terrainShapeID, int regionSizeX,
                         int regionSizeY, float rowSpacing, float columnSpacing,
                         float * posts);

   /// Add a joint between two actors.
   ///
   /// @param jointID The unique identifier of the joint being added.
   /// @param actorID1 The unique identifier of the first actor being joined.
   /// @param actorID2 The unique identifier of the second actor being joined.
   /// @param actorPos1 The position of joint relative to the first actor.
   /// @param actorQuat1 The orientation of joint relative to the first actor.
   /// @param actorPos2 The position of joint relative to the second actor.
   /// @param actorQuat2 The orientation of joint relative to the second actor.
   /// @param linearLowerLimit Lower limits of each of the 3 translation axes.
   /// @param linearUpperLimit Upper limits of each of the 3 translation axes.
   /// @param angularLowerLimit Lower limits of each of the 3 rotational axes.
   /// @param angularUpperLimit Upper limits of each of the 3 rotational axes.
   ///
   void   addJoint(unsigned int jointID, unsigned int actorID1,
                   unsigned int actorID2, float * actor1Pos, float * actor1Quat,
                   float * actor2Pos, float * actor2Quat,
                   float * linearLowerLimit, float * linearUpperLimit,
                   float * angularLowerLimit, float * angularUpperLimit);

   /// Add a joint between an actor and the global frame.
   ///
   /// @param jointID The unique identifier of the joint being added.
   /// @param actorID The unique identifier of the joint.
   /// @param actorPos The position of joint relative to the actor.
   /// @param actorQuat The orientation of joint relative to the actor.
   /// @param linearLowerLimit Lower limits of each of the 3 translation axes.
   /// @param linearUpperLimit Upper limits of each of the 3 translation axes.
   /// @param angularLowerLimit Lower limits of each of the 3 rotational axes.
   /// @param angularUpperLimit Upper limits of each of the 3 rotational axes.
   ///
   void   addGlobalFrameJoint(unsigned int jointID, unsigned int actorID,
                              float * actorPos, float * actorQuat,
                              float * linearLowerLimit,
                              float * linearUpperLimit,
                              float * angularLowerLimit,
                              float * angularUpperLimit);


   /// Remove joint from the physics scene.
   ///
   /// @param id The unique identifier of the PhysX joint
   void   removeJoint(unsigned int id);

   /// This method runs the main simulation of PhysX and will be called at
   /// every frame of the simulator.
   ///
   /// @param time The amount of time that the PhysX world should simulate.
   /// @param updatedEntityCount Passed by reference value that returns the
   /// number of entities that have updated values.
   /// @param updatedCollisionCount Passed by reference value that returns the
   /// number of collisions that have occurred.
   ///
   void   simulate(float time, unsigned int * updatedEntityCount,
      unsigned int * updatedCollisionCount);

   /// Construct a joint between two actors.
   ///
   /// @param jointID The unique identifier of the joint being added
   /// @param actor1 The first actor being joined.
   /// @param actor2 The second actor being joined.
   /// @param actorPos1 The position of joint relative to the first actor
   /// @param actorQuat1 The orientation of joint relative to the first actor
   /// @param actorPos2 The position of joint relative to the second actor
   /// @param actorQuat2 The orientation of joint relative to the second actor
   /// @param linearLowerLimit Lower limits of each of the 3 translation axes
   /// @param linearUpperLimit Upper limits of each of the 3 translation axes
   /// @param angularLowerLimit Lower limits of each of the 3 rotational axes
   /// @param angularUpperLimit Upper limits of each of the 3 rotational axes
   ///
   void   constructJoint(unsigned int jointID, PhysXRigidActor * actor1,
                         PhysXRgidiActor * actor2, float * actor1Pos,
                         float * actor1Quat, float * actor2Pos,
                         float * actor2Quat, float * linearLowerLimit,
                         float * linearUpperLimit, float * angularLowerLimit,
                         float * angularUpperLimit);
}
