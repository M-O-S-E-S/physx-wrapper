
/// This is a header file for the PhysXLib.c++ class. It will not actually be 
/// included inside of any file and is used for doxygen commenting. It is also
/// the main class of the PhysXWrapper and used as the bridge to link c# with
/// c++.

/// The PhysX foundation that will be used throughout the PhysXWrapper.
static PxFoundation * px_foundation;

/// The PxPhysics object that will be used throughout the PhysXWrapper.
static PxPhysics * px_physics;

/// The PxScene that will hold the entire physical scene.
static PxScene * px_scene;

/// The PxCooking object that is used when creating the height field.
static PxCooking * px_cooking;

/// The default error callback of PhysX used when creating the foundation.
static PxDefaultErrorCallback error_callback;

/// The default allocator of PhysX used when creating the foundation.
static PxDefaultAllocator allocator_callback;

/// Object that will handle the receiving and storage of all collisions within
/// the physical scene.
static PhysXCollisionCallback * px_collisions;

/// Rigid static physical object that will hold the ground plane of the
/// physical scene.
static PxRigidStatic * ground_plane;

/// Flag that tracks when the scene has been initialized.
static int scene_initialized;

/// The max number of updates that can be passed from PhysXWrapper.
/// This number is based on the size of the pinned memory array responsible for
/// passing updates from the PhysXWrapper.
static int max_updates;

/// Map of all the actors in the current scene with keys being equal to the
/// actor's id stored inside an atInt and the value being the actor.
static atMap * actor_map;

/// PvdConnection used to establish a stream with PhysX visual debugger.
static debugger::comm::PvdConnection * theConnection;

/// A global value to store a specific ID for the terrain since none will be
/// provided.
static atInt * terrain_id;

/// The height field scale that will be used to adjust the height field to an
/// integer from a float and vice a versa.
static float height_field_scale;

/// Array of updates that is shared.
static EntityProperties * update_array;

/// Array of collisions that is shared.
static CollisionProperties * collisions_array;


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
PhysXRigidActor * createActor(unsigned int id, char * name, float x, float y,
   float z, bool isDynamic);

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
PhysXRigidActor * createActor(unsigned int id, char * name, float x, float y,
   float z, PxQuat Rot, bool isDynamic);

/// Method to fetch the actor from the map of actors.
/// 
/// @param id The id of the actor that is being fetched.
///
/// @return The actor with the given id or null if the actor was not inside of
/// the map of actors.
///
PhysXRigidActor * getActor(unsigned int id);

/// Method to fetch the actor from the map of actors.
/// 
/// @param id The id of the actor that is being fetched.
///
/// @return The actor with the given id or null if the actor was not inside of
/// the map of actors.
///
PhysXRigidActor * getActor(atInt * id);

/// Custom filter shader used for collision filtering and to customize the
/// collection of flags describing the actions to take on a collision pair.
///
/// TODO: I don't know the parameters so either Chandler or Rob will have to
/// make a pass over this and add in the parameters and return value.
///
PxFilterFlags contactFilterShader(PxFilterObjectAttributes attributes0,
   PxFilterData filterData0, PxFilterObjectAttributes attributes1,
   PxFilterData filterData1, PxPairFlags& pairFlags, 
   const void * constantBlock, PxU32 constantBlockSize);

/// Method to connect the PhysXWrapper to a running PhysX Visual Debugger.
///
void startVisualDebugger();


extern "C"
{
   /// Initializes the foundation, physics, cooking, collisions, and visual
   /// debugger.
   ///
   /// @return 1 if successfully initialized and 0 otherwise.
   ///
   int initialize();

   /// Cleans up the PhysXWrapper by releasing the visual debugger, physics,
   /// and foundation in that order.
   ///
   void release();

   /// Initialize the update array for updating the physical object properties
   /// after every simulate call.
   /// 
   /// @param updateArray The array that has been pinned to memory and will be 
   /// transfering the updates from the unmanaged code to managed code.
   /// @param maxUpdates The size of the updateArray which in turn determines
   /// how many updates can be sent after each simulate call.
   ///
   void initEntityUpdate(EntityProperties * updateArray, int maxUpdates);

   /// Initialize the collision array for updating the physical object 
   /// collisions after every simulate call.
   /// 
   /// @param collisionArray The array that has been pinned to memory and will 
   /// be transfering the collisions from the unmanaged code to managed code.
   /// @param maxUpdates The size of the collisionArray which in turn determines
   /// how many collisions can be sent after each simulate call.
   ///
   void initCollisionUpdate(CollisionProperties * collisionArray, 
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
   void createScene(bool gpuEnabled, bool cpuEnabled, int cpuMaxThreads);

   /// Call the scene release to clean up the scene.
   /// 
   void releaseScene();

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
   void createActorSphere(unsigned int id, char * name, float x, float y, 
      float z, float staticFriction, float dynamicFriction, float restitution,
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
   void createActorBox(unsigned int id, char * name, float x, float y, 
      float z, float staticFriction, float dynamicFriction, float restitution,
      float halfX, float halfY, float halfZ, float density, bool isDynamic);

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
   void createActorCapsule(unsigned int id, char * name, float x, float y, 
      float z, float staticFriction, float dynamicFriction, float restitution,
      float halfHeight, float radius, float density, bool isDynamic);

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
   void createActorTriangleMesh(unsigned int id, char * name, float x, float y, 
      float z, float staticFriction, float dynamicFriction, float restitution,
      float * vertices, float * indices, int vertexCount, int indexCount, 
      bool isDynamic);

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
   /// @param staticFriction The static friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param dynamicFriction The dynamic friction used for this actor when
   /// sliding against other actors in the physical scene.
   /// @param restitution How bouncy this actor is when colliding with other
   /// actors in the physical scene.
   /// @param vertices The list of vertices of the mesh that make up the 
   /// convex mesh.
   /// @param vertexCount The size of the list of vertices.
   /// @param isDynamic Flag that determines if this is a static or dynamic
   /// actor.
   ///
   void createActorTriangleMesh(unsigned int id, char * name, float x, float y, 
      float z, float staticFriction, float dynamicFriction, float restitution,
      float * vertices, int vertexCount, bool isDynamic);

   /// Remove an actor from the physical scene.
   ///
   /// @param id The id of the actor to be removed.
   ///
   void removeActor(unsigned int id);

   /// Updates an actors position and orientation inside the physical scene.
   ///
   /// @param id The id of the actor to be updated.
   /// @param posX The x value of the new physical translation in the scene.
   /// @param posY The y value of the new physical translation in the scene.
   /// @param posZ The z value of the new physical translation in the scene.
   /// @param rotX The x value of the quaternion representing the new roatation
   /// of the actor in the scene.
   /// @param rotY The y value of the quaternion representing the new roatation
   /// of the actor in the scene.
   /// @param rotZ The z value of the quaternion representing the new roatation
   /// of the actor in the scene.
   /// @param rotW The w value of the quaternion representing the new roatation
   /// of the actor in the scene.
   ///
   void setTranslation(unsigned int id, float posX, float posY, float posZ,
      float rotX, float rotY, float rotZ, float rotW);

   /// Updates an actors position inside the physical scene.
   ///
   /// @param id The id of the actor to be updated.
   /// @param x The x value of the new physical translation in the scene.
   /// @param y The y value of the new physical translation in the scene.
   /// @param z The z value of the new physical translation in the scene.
   ///
   void setPosition(unsigned int id, float x, float y, float z);

   /// Method to fetch the current position of an actor in the physical scene.
   ///
   /// @param id The id of the actor to be fetched.
   ///
   /// @return An array of floats with the x, y, z values for the position of
   /// the actor in the physical scene.
   ///
   float * getPosition(unsigned int id);

   /// Updates an actors rotation inside the physical scene.
   ///
   /// @param id The id of the actor to be updated.
   /// @param x The x value of the quaternion representing the new rotation of
   /// the actor in the physical scene.
   /// @param y The y value of the quaternion representing the new rotation of
   /// the actor in the physical scene.
   /// @param z The z value of the quaternion representing the new rotation of
   /// the actor in the physical scene.
   /// @param w The w value of the quaternion representing the new rotation of
   /// the actor in the physical scene.
   ///
   void setRotation(unsigned int id, float x, float y, float z, float w);

   /// Method to fetch the current orientation of an actor in the physical 
   /// scene.
   ///
   /// @param id The id of the actor to be fetched.
   ///
   /// @return An array of floats with the x, y, z, w values for the quaternion
   /// representing the orientation of the actor in the physical scene.
   ///
   float * getRotation(unsigned int id);

   /// Updates an actor with a new linear velocity.
   ///
   /// @param id The id of the actor to be updated.
   /// @param x The velocity of the actor in the x direction.
   /// @param y The velocity of the actor in the y direction.
   /// @param z The velocity of the actor in the z direction.
   ///
   void setLinearVelocity(unsigned int id, float x, float y, float z);

   /// Updates an actor with a new angular velocity.
   ///
   /// @param id The id of the actor to be updated.
   /// @param x The velocity of the actor in the x direction.
   /// @param y The velocity of the actor in the y direction.
   /// @param z The velocity of the actor in the z direction.
   ///
   void setAngularVelocity(unsigned int id, float x, float y, float z);

   /// Updates the scene gravity on an actor to the new values.
   ///
   /// @param id The id of the actor to be updated.
   /// @param x The amount of gravity applied to the actor in the x direction.
   /// @param y The amount of gravity applied to the actor in the y direction.
   /// @param z The amount of gravity applied to the actor in the z direction.
   ///
   void setGravity(unsigned int id, float x, float y, float z);

   /// Enable or disable gravity on an actor.
   ///
   /// @param id The id of the actor that gravity is being changed.
   /// @param enabled Flag that determines if gravity is being enabled or
   /// disabled for this particular actor.
   ///
   void enableGravity(unsigned int id, bool enabled);

   /// Method to create a ground plane to prevent actors from falling forever.
   /// 
   /// @param x The x value of the position where the plane should be created.
   /// @param y The y value of the position where the plane should be created.
   /// @param z The z value of the position where the plane should be created.
   ///
   void createGroundPlane(float x, float y, float z);

   /// Release the resources currently be used by the ground plane.
   ///
   void releaseGroundPlane();

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
   void setHeightField(int terrainShapeID, int regionSizeX, int regionSizeY, 
      float rowSpacing, float columnSpacing, float * posts);

   /// This method runs the main simulation of PhysX and will be called at
   /// every frame of the simulator.
   ///
   /// @param time The amount of time that the PhysX world should simulate.
   /// @param updatedEntityCount Passed by reference value that returns the
   /// number of entities that have updated values.
   /// @param updatedCollisionCount Passed by reference value that returns the
   /// number of collisions that have occurred.
   ///
   void simulate(float time, unsigned int * updatedEntityCount, 
      unsigned int * updatedCollisionCount);
}
