## [0.5.1-preview.2] - 2020-10-14

### Upgrade guide

### Changes

* Dependencies
    * Updated Burst from `1.3.2` to `1.3.7`
    * Updated Mathematics from `1.1.0` to `1.2.1`
    * Updated Collections from `0.11.0-preview.17` to `0.14.0-preview.16`
    * Updated Entities from `0.13.0-preview.24` to `0.16.0-preview.21`
    * Updated Jobs from `0.4.0-preview.18` to `0.7.0-preview.17`

* Run-Time API
    * Added the `TerrainCollider.Filter` setter
    * Removed the `CompoundCollider.Filter` setter as it was doing the wrong thing (composite collider filters should be the union of their children)
    * Added `BuildPhysicsWorld.AddDependencyToComplete()` which takes a job dependency that the `BuildPhysicsWorld` system should complete immediately in its `OnUpdate()` call (before scheduling new jobs). The reason is that this system does reallocations in the `OnUpdate()` immediately (not in jobs), and any previous jobs that are being run before this system could rely on that data being reallocated. This way, these jobs can provide their dependency to `BuildPhysicsWorld` and make sure it will wait until they are finished before doing the reallocations.
	* Added the option to provide a custom explosion filter in 'PhysicsVelocity.ApplyExplosionForce'.

* Authoring/Conversion API

* Run-Time Behavior
    * Changed Graphical Interpolation default to simpler implementation that doesn't try and consider velocities
    * `BuildPhysicsWorld.CreateMotions` now gives Kinematic bodies a zero Gravity Factor (i.e. they will not be affected by gravity)
    * Setting the `Collider.Filter` is now allowed for Terrain colliders as well, as opposed to previously only working for Convex colliders

* Authoring/Conversion Behavior

### Fixes
* Fixed the potential issues if more than one job implements IBodyPairsJob.
* DebugStream.DrawComponent now cleans up its associated GameObject
* Fixed a bug in 'PhysicsVelocity.ApplyExplosionForce' where the provided collider's collision filter could prevent the explosion from happening.
* Fixed a memory leak in the Collider debug display gizmo

### Known Issues

## [0.5.0-preview.1] - 2020-09-15

### Upgrade guide

* Physics systems now update in the `FixedStepSimulationSystemGroup`, using a fixed timestep provided by the group. This ensures that the results of the physics simulation do not depend on the application's display frame rate. Important consequences for applications:
    * Application systems that need to run at the same rate as the physics systems should also be moved into `FixedStepSimulationSystemGroup` (e.g., systems that handle collision or trigger events).
    * Application systems that should continue to run once per display frame should remove any `[UpdateAfter]` attributes targeting the physics systems. Any update order constraints for such systems should instead be expressed with respected to `FixedStepSimulationSystemGroup`.
    * The DOTS fixed timestep is get/set using the `Timestep` property on the `FixedStepSimulationSystemGroup`. The group's default timestep value is 1/60 second. The group's timestep is not affected by changes to the fixed timestep specified in the Project Settings. If this behavior is desired, applications can set `FixedStepSimulationSystemGroup.Timestep` to `UnityEngine.Time.fixedDeltaTime` every frame.
    * Care must be taken when processing input events during the FixedStepSimulationSystemGroup's update. The group may update zero, or one, or many times per display frame. If input events are polled once per display frame, naive input processing may lead to input events being skipped or processed multiple times.
    * If your application's frame rate is faster than the fixed timestep, rigid bodies may appear to move in stop motion by default (as with classic GameObject-base physics). You can enable smoothing options for their graphics representations as needed.
        * Application systems that must update once per display frame, yet which query the `Translation` or `Rotation` of dynamic rigid bodies, should instead query `LocalToWorld` when smoothed graphical transformations are required.
* It is possible to see integrity failures when updating to this release. `BuildPhysicsWorld` and `ExportPhysicsWorld` expect the chunk layouts for rigid bodies to be the same at both ends of the physics pipeline in order to write the simulation results to component data. These messages indicate that you are making some structural change or modifying rigid bodies' component data (e.g., `Translation`, `Rotation`, `PhysicsCollider`) between these systems.
* Old, unused serialized data have been removed from `PhysicsShapeAuthoring` and `PhysicsMaterialTemplate`. Ensure you have run the upgrade utility in your project before updating to this version of the package (Window -> DOTS -> Physics -> Upgrade Data).

### Changes

* Dependencies
    * Updated minimum Unity Editor version from `2019.4.0f1` to `2020.1.0f1`
    * Updated Burst from `1.3.0` to `1.3.2`
    * Updated Collections from `0.9.0-preview.6` to `0.11.0-preview.17`
    * Updated Entities from `0.11.1-preview.4` to `0.13.0-preview.24`
    * Updated Jobs from `0.2.10-preview.12` to `0.4.0-preview.18`
    * Updated Performance Testing API from `2.0.8-preview` to `2.2.0-preview`

* Run-Time API
    * Added the following new types:
        * `PhysicsGraphicsIntegration` namespace
            * `PhysicsGraphicalSmoothing` and `PhysicsGraphicalInterpolationBuffer` components, which can be used to smooth a rigid body's graphical motion when rendering and physics are out of sync
            * `RecordMostRecentFixedTime`, which stores time values from the most recent tick of `FixedStepSimulationSystemGroup`
            * `BufferInterpolatedRigidBodiesMotion`, which stores dynamic rigid bodies' motion properties at the start of the frame when when smoothed bodies use interpolation
            * `CopyPhysicsVelocityToSmoothing`, which stores dynamic rigid bodies' velocities after physics has finished in the current frame
            * `SmoothRigidBodiesGraphicalMotion`, which writes dynamic rigid bodies' `LocalToWorld` when smoothing is enabled
            * `GraphicalSmoothingUtility` class, which contains various methods for smoothing the motion of rigid bodies
    * Renamed `ComponentExtensions` to `PhysicsComponentExtensions`
    * Changed the following members/types:
        * `PhysicsComponentExtensions.GetCenterOfMassWorldSpace()` now passes `PhysicsMass` as `in` rather than `ref`.
        * `PhysicsComponentExtensions.GetLinearVelocity()` now passes `PhysicsVelocity` as `in`.
        * `PhysicsWorldExtensions.CalculateVelocityToTarget()` is now implemented and passes a `RigidTransform` for the target rather than a separate `float3` and `quaternion`.
    * Removed the following expired members/types:
        * `BodyIndexPair.BodyAIndex`
        * `BodyIndexPair.BodyBIndex`
        * Body pair interfaces on events and modifiers (`CollisionEvent`, `TriggerEvent`, `ModifiableContactHeader`, `ModifiableBodyPair` and `ModifiableJacobianHeader`)
            * `BodyCustomTags`
            * `BodyIndices`
            * `ColliderKeys`
            * `Entities`
        * `ComponentExtensions.GetCenterOfMass()`
        * `ComponentExtensions.SetCenterOfMass()`
        * `ComponentExtensions.GetAngularVelocity()`
        * `ComponentExtensions.SetAngularVelocity()`
        * `EndFramePhysicsSystem.HandlesToWaitFor`
        * `FinalJobHandle` on all core physics systems (`BuildPhysicsWorld`, `StepPhysicsWorld`, `ExportPhysicsWorld` and `EndFramePhysicsSystem`)
        * `Joint.JointData`
        * `JointData`
        * `JointFrame`
        * `Material.IsTrigger`
        * `Material.EnableCollisionEvents`
        * `MotionData.GravityFactor`
        * `PhysicsJoint.JointData`
        * `PhysicsJoint.EntityA`
        * `PhysicsJoint.EntityB`
        * `PhysicsJoint.EnableCollision`
        * `SimulationContext.Reset()` passing `PhysicsWorld`
        * `Solver.ApplyGravityAndCopyInputVelocities()` passing `NativeArray<MotionData>`
        * `Solver.SolveJacobians()` not passing explicit `StabilizationData`
    * Added `JointType.LimitedDegreeOfFreedom` and associated creation and control functions
	* Added `Aabb.Intersect()` function
	* Added `PhysicsComponentExtensions.GetEffectiveMass()`

* Authoring/Conversion API
    * Removed the following expired members/types:
        * `LegacyJointConversionSystem` (now internal)
        * `PhysicsMaterialTemplate.IsTrigger`
        * `PhysicsMaterialTemplate.RaisesCollisionEvents`
        * `PhysicsShapeAuthoring.IsTrigger` and `OverrideIsTrigger`
        * `PhysicsShapeAuthoring.RaisesCollisionEvents` and `OverrideRaisesCollisionEvents`
    * Added the following new types:
        * `BodySmoothing` enum
    * Added the following new members:
        * `PhysicsBodyAuthoring.Smoothing`, to enable motion smoothing
    * `PhysicsBodyAuthoring.LinearDamping` and `AngularDamping` setters now clamp incoming values to be at least 0.

* Run-Time Behavior
    * Debug rendering is now significantly faster, in the case of 3D lines.
    * Physics systems now update in the `FixedStepSimulationSystemGroup`.

* Authoring/Conversion Behavior
    * Classic `Rigidbody` interpolation mode is now supported during conversion.
    * Inspector help button for built-in authoring components and assets now opens the corresponding page in the API reference.

### Fixes
* Fixed issue where orientation in Physics Shape component would get dirtied when nothing changed
* Fixed issue with `PhysicsJoint.CreateHinge()` only working on a single axes.
* Fixed `Constraint.Dimension` not returning 0 when it should.
* Reduced size of compound collider/mesh collider AABB in cases where their bodies are rotated by some angle.
* Added optional integrity checks for physics ECS data consistency between BuildPhysicsWorld and ExportPhysicsWorld. Checks can be run in editor only.
* Fixed regression that caused additional sub-meshes to be ignored when converting mesh colliders.

### Known Issues
* Physics debug display may not work properly while stepping frame by frame in Editor.

## [0.4.1-preview] - 2020-07-28

### Changes

* Run-Time API
    * Added the following members:
        * `FloatRange.Mid`
        * `AABB.ClosestPoint`
    * Changed the following members/types:
        * All systems now inherit `SystemBase` instead of `ComponentSystem`.

### Fixes

* When using Unity 2020.1.0b13 or newer, it is now possible to convert mesh colliders inside of sub-scenes when their input meshes do not have read/write enabled. Meshes converted at run-time must still have read/write enabled.
* Stopped emitting warning messages about physics material properties being upgraded when creating new objects from editor scripts.
* Fixed warnings from exceptions thrown in Bursted code paths when using Burst 1.4.0.
* Fixed issue with static layer not being rebuilt when order of entities in chunk changes.
* Fixed issue with invalid colliders with empty AABB breaking bounding volume hierarchy and making objects "disappear" from the world.
* Fixed an editor crash when maximum level of composite collider nesting is breached.

## [0.4.0-preview.5] - 2020-06-18

### Upgrade guide

* `RigidBody` queries may no longer work, since they required inputs to be transformed to body space. As of this release, body queries require input in world space.
* Core physics systems now expose `AddInputDependency()` and `GetOutputDependency()` methods for user code to be able to plug in user systems between any of the core physics systems (`BuildPhysicsWorld`, `StepPhysicsWorld`, `ExportPhysicsWorld` and `EndFramePhysicsSystem`).
* The serialization layout has changed for `PhysicsShapeAuthoring` and `PhysicsMaterialTemplate`. Because data are upgraded before Prefab overrides are applied, you must manually re-apply any Prefab overrides that existed on scene objects, nested Prefabs, or Prefab variants for the migrated properties. It is recommended you use the data upgrade utility under the menu item Window -> DOTS -> Physics -> Upgrade Data.
    * `m_IsTrigger` and `m_RaisesCollisionEvents` have been replaced with `m_CollisionResponse`.
    * `m_BelongsTo` has been replaced with `m_BelongsToCategories`
    * `m_CollidesWith` has been replaced with `m_CollidesWithCategories`
    * `m_CustomTags` has been replaced with `m_CustomMaterialTags`

### Changes

* Dependencies
    * Updated minimum Unity Editor version from `2019.3.0f1` to `2019.4.0f1`
    * Updated Burst from `1.3.0-preview.7` to `1.3.0`
    * Updated Collections from `0.7.1-preview.3` to `0.9.0-preview.6`
    * Updated Entities from `0.9.0-preview.6` to `0.11.1-preview.4`
    * Updated Jobs from `0.2.8-preview.3` to `0.2.10-preview.12`
    * Updated Performance Testing API from `1.3.3-preview` to `2.0.8-preview`

* Run-Time API
    * Added the following new types:
        * `BuildPhysicsWorld.CollisionWorldProxyGroup`
        * `CollisionResponsePolicy` (introduces a new value, `None`, which allows a shape to move and participate in queries, without generating a collision response or overlap events)
        * `CollisionWorldProxy` to synchronize `CollisionWorld` with data flow graph node sets
        * `ColliderCastNode` to perform queries in a data flow graph node
        * `ForceMode` to use with extension methods analogous to those used with classic `Rigidbody`
        * `JointComponentExtensions` for use with `PhysicsJoint`
        * `JointType` to serve as a hint to code modifying joint limits
        * `PhysicsExclude` to easily exclude bodies from physics temporarily
        * `PhysicsMassOverride` to easily make dynamic bodies enter and exit a kinematic state
        * `PhysicsConstrainedBodyPair`
        * `PhysicsJointCompanion` to keep track of sets of joints used to stabilize complex joint configurations
        * `RaycastNode` to perform queries in a data flow graph node
        * `Solver.StabilizationData`
        * `Solver.StabilizationHeuristicSettings`
    * Added the following members:
        * `ComponentExtensions.ApplyExplosionForce()` equivalent to classic `Rigidbody.ApplyExplosionForce()` method.
        * `ComponentExtensions.GetImpulseFromForce()` to map `ForceMode` variants to impulses.
        * `FloatRange.Sorted()` to ensure `Min` is not larger than `Max`.
        * `PhysicsStep.SolverStabilizationHeuristicSettings` to enable and configure solver stabilization heuristic. This can improve behavior in stacking scenarios, as well as overall stability of bodies and piles, but may result in behavior artifacts. It is off by default to avoid breaking existing behavior. Setting it to true enables the heuristic and its default parameters.
        * `PhysicsStep.SynchronizeCollisionWorld` to enable rebuild of the dynamic bodies bounding volume hierarchy after the step. This enables precise query results before the next `BuildPhysicsWorld` update call. Note that `BuildPhysicsWorld` will do this work on the following frame anyway, so only use this option when another system must know about the results of the simulation before the end of the frame (e.g., to destroy or create some other body that must be present in the following frame). In most cases, tolerating a frame of latency is easier to work with and is better for performance.
        * `PhysicsVelocity.CalculateVelocityToTarget()` to move kinematic bodies to desired target locations without teleporting them.
        * `SurfaceConstraintInfo.IsMaxSlope`
        * Implemented `ToString()` on some types
            * `ColliderCastHit`
            * `ColliderCastInput`
            * `ColliderKey`
            * `RaycastHit`
            * `RaycastInput`
    * Renamed the following members/type:
        * `BodyAIndex` and `BodyBIndex` are now `BodyIndexA` and `BodyIndexB` across the codebase for consistency.
        * `ComponentExtensions.GetAngularVelocity()` and `SetAngularVelocity()` are now `GetAngularVelocityWorldSpace()` and `SetAngularVelocityWorldSpace()`, respectively.
        * `ComponentExtensions.GetCenterOfMass()` and `SetCenterOfMass()` are now `GetCenterOfMassWorldSpace()` and `SetCenterOfMassWorldSpace()`.
        * `JointFrame` is now `BodyFrame`
    * Changed the following members/types:
        * Replaced all usages of `NativeSlice` with `NativeArray`.
        * Replaced pair interfaces on events and modifiers (`CollisionEvent`, `TriggerEvent`, `ModifiableContactHeader`, `ModifiableBodyPair` and `ModifiableJacobianHeader`) with direct accessors:
            * `EntityPair` is now `EntityA` and `EntityB`
            * `BodyIndexPair` is now `BodyAIndex` and `BodyBIndex`
            * `ColliderKeyPair` is now `ColliderKeyA` and `ColliderKeyB`
            * `CustomTagsPair` is now `CustomTagsA` and `CustomTagsB`
        * `Material.MaterialFlags` is now internal. Access flags through individual properties (`CollisionResponse`, `EnableMassFactors` and `EnableSurfaceVelocity`).
        * `Joint` is now a fixed size.
            * `Joint.EnableCollision` is now `byte` instead of `int`.
            * `Joint.JointData` has been deprecated. Use `AFromJoint`, `BFromJoint`, `Constraints`, and `Version` instead.
        * `PhysicsJoint` is now mutable.
        * `PhysicsJoint.CreatePrismatic()` factory does not take a `distanceFromAxis` parameter (in contrast to `JointData` factory)
        * `PhysicsJoint.CreateRagdoll()` factory now takes perpendicular angular limits in the range (-pi/2, pi/2) instead of (0, pi) in old `JointData` factory.
    * Deprecated the following members/types:
        * `EndFramePhysicsSystem.HandlesToWaitFor` (use `AddInputDependency()` instead).
        * `FinalJobHandle` on all core physics systems (`BuildPhysicsWorld`, `StepPhysicsWorld`, `ExportPhysicsWorld` and `EndFramePhysicsSystem`) (use `GetOutputDependency()` instead).
        * `JointData`
        * `Material.IsTrigger` (use `CollisionResponse` instead)
        * `Material.EnableCollisionEvents` (use `CollisionResponse` instead)
        * `PhysicsJoint.EntityA` (now defined on `PhysicsConstrainedBodyPair`)
        * `PhysicsJoint.EntityB` (now defined on `PhysicsConstrainedBodyPair`)
        * `PhysicsJoint.EnableCollision` (now defined on `PhysicsConstrainedBodyPair`)
        * `PhysicsJoint.JointData` (use `BodyAFromJoint`, `BodyBFromJoint`, and `GetConstraints()`/`SetConstraints()` instead).
        * `SimulationContext.Reset()` passing `PhysicsWorld` (use signature passing `SimulationStepInput` instead).
    * Removed the following members/types:        
        * `CollisionFilter.IsValid`
        * `CollisionWorld.ScheduleUpdateDynamicLayer()`
        * `Constraint` factory signatures passing `float` values for ranges:
            * `Cone()`
            * `Cylindrical()`
            * `Planar()`
            * `Twist()`
        * `ISimulation.ScheduleStepJobs()` signature without callbacks and thread count hint (as well as all implementations)
        * `JointData` factory signatures passing `float3` and `quaternion` pairs for joint frames, as well as `Constraint[]`:
            * `CreateFixed()`
            * `CreateHinge()`
            * `CreateLimitedHinge()`
            * `CreatePrismatic()`
            * `CreateRagdoll()`
        * `MassFactors.InvInertiaAndMassFactorA`
        * `MassFactors.InvInertiaAndMassFactorB`
        * `MotionData.GravityFactor` (use `MotionVelocity.GravityFactor` instead)
        * `MotionVelocity.InverseInertiaAndMass`
        * `RigidBody.HasCollider`
        * `SimplexSolver.Solve()` signature passing `PhysicsWorld`
        * `SimulationStepInput.ThreadCountHint`

* Authoring/Conversion API
    * Added the following types
        * `BeginJointConversionSystem`
        * `EndJointConversionSystem` (allows other conversion systems to find joint entities created during conversion)
        * `PhysicsMaterialTemplate.CollisionResponse`
        * `PhysicsShapeAuthoring.CollisionResponse`
        * `PhysicsShapeAuthoring.OverrideCollisionResponse`
    * Added the following members
        * `PhysicsStepAuthoring.EnableSolverStabilizationHeuristic` to enable solver stabilization heuristic with default settings
        * `PhysicsStepAuthoring.SynchronizeCollisionWorld`
    * Deprecated the following members/types:
        * `LegacyJointConversionSystem` has been marked obsolete and will be made internal in the future. Use `BeginJointConversionSystem` and `EndJointConversionSystem` to schedule system updates as needed.
        * `PhysicsMaterialTemplate.IsTrigger` (use `CollisionResponse` property instead)
        * `PhysicsMaterialTemplate.RaisesCollisionEvents` (use `CollisionResponse` property instead)
        * `PhysicsShapeAuthoring.IsTrigger` (use `CollisionResponse` property instead)
        * `PhysicsShapeAuthoring.RaisesCollisionEvents` (use `CollisionResponse` property instead)
        * `PhysicsShapeAuthoring.OverrideIsTrigger` (use `OverrideCollisionResponse` property instead)
        * `PhysicsShapeAuthoring.OverrideRaisesCollisionEvents` (use `OverrideCollisionResponse` property instead)
    * Removed the following expired members:
        * `LegacyColliderConversionSystem.ProduceMaterial()` (removed without expiration; class is not intended to be sub-classed outside the package)
        * `PhysicsShapeAuthoring.GetCapsuleProperties()` signature returning `CapsuleGeometry`
        * `PhysicsShapeAuthoring.SetCapsule()` signature passing `CapsuleGeometry`

* Run-Time Behavior
    * `BuildPhysicsWorld.JointEntityGroup` now requires both a `PhysicsJoint` and `PhysicsConstrainedBodyPair` component.
    * `Constraint` factory methods taking a `FloatRange` swizzle the input value if needed to ensure that `Min` cannot be greater than `Max`.
    * `RigidBody` queries (`Raycast()`, `ColliderCast()`, `PointDistance()`, and `ColliderDistance()`) now use world space input, and provide world space output, instead of using body space.     

* Authoring/Conversion Behavior
    * It is now possible to create a compound collider by adding multiple `PhysicsShapeAuthoring` components to a single GameObject.
    * It is now possible to convert a GameObject with multiple `Joint` components on it.

### Fixes

* When using Unity 2019.3.1f1 and newer, making changes to a `PhysicsMaterialTemplate` asset or a `Mesh` asset will now trigger a reimport of any sub-scenes containing shapes that reference it.
* `IBodyPairsJob` now skips all joint pairs, when previously it didn't skip the joint pair if it was the first one in the list.
* Fixed the issue of the `CompoundCollider` with no colliding children (all children are triggers) having invalid mass properties.
* Fixed a potential race condition when `SynchronizeCollisionWorld` was set to true in the `SimulationStepInput`.
* `CollisionFilter` and `Advanced` material properties in the Inspector can now be expanded by clicking the label.

## [0.3.2-preview] - 2020-04-16

### Upgrade guide

* All `PhysicsShapeAuthoring` components that were newly added after version 0.3.0 were incorrectly initialized to have a bevel radius of 0. It is recommended that you audit recently authored content to assign reasonable non-zero values.

### Changes

* Dependencies
    * Updated Collections from `0.5.2-preview.8` to `0.7.1-preview.3`
    * Updated Entities from `0.6.0-preview.24` to `0.9.0-preview.6`
    * Updated Jobs from `0.2.5-preview.20` to `0.2.8-preview.3`

* Run-Time Behavior
    * In order to test gameplay end to end determinism users should:
        * Navigate to UnityPhysicsEndToEndDeterminismTest.cs (and HavokPhysicsEndToEndDeterminsmTest.cs if using HavokPhysics)
        * Remove #if !UNITY_EDITOR guards around [TestFixture] and [UnityTest] attributes (alternatively, users can run the tests
          in standalone mode without the need to remove #ifs)
        * Add scene that needs to be tested (File -> Build Settings -> Add Open Scenes)
        * Make sure that synchronous burst is enabled
        * Run the test in Test Runner

* Authoring/Conversion Behavior
    * Compound conversion system is now deterministic between runs.

### Fixes

* Volume of dynamic meshes is now being approximated with the volume of the mesh AABB, as opposed to previously being set to 0.
* Fixed regression causing newly added `PhysicsShapeAuthoring` components to initialize with a bevel radius of 0.

### Known Issues

## [0.3.1-preview] - 2020-03-19

### Upgrade guide

 * User implemented query collectors (`ICollector<T>`) may no longer work. The reason is that `ICollector<T>.TransformNewHits()` was removed. To get the collectors working again, move all logic from
 `ICollector<T>.TransformNewHits()` to `ICollector<T>.AddHit()`. All the information is now available in `ICollector<T>.AddHit()`. Also, `IQueryResult.Transform()` was removed, and user implementations of it will not get called anywhere in the engine.

### Changes

* Dependencies
    * Updated Burst from `1.3.0-preview.3` to `1.3.0-preview.7`

* Run-Time API
    * The following properties are added to `IQueryResult` interface:
        * `RigidBodyIndex`
        * `ColliderKey`
        * `Entity`
    * `ICollector.AddHit()` now has all the information ready to perform custom logic, instead of waiting for TransformNewHits().
    * Removed `Transform()` from `IQueryResult` interface and from all its implementations.
    * Removed both `TransformNewHits()` methods from `ICollector` interface and from all its implementations. All the information is now ready in `ICollector.AddHit()`.

### Fixes

* Setting `Collider.Filter` now increments the header version so that the simulation backends can recognise the change.
* Asking for collision/trigger events in scenes with no dynamic bodies no longer throws errors.
* Updated to new version of Burst, which fixes a regression that caused `ConvexCollider.Create()` to produce hulls with a very small bevel radius.
* DOTS Run-time failures due to multiple inheritance of jobs have now been fixed.
* Changed `Math.IsNormalized` to use a larger tolerance when comparing float3 length.

## [0.3.0-preview.1] - 2020-03-12

### Upgrade guide

* In order to lead to more predictable behavior and allow for repositioning instantiated prefabs, static bodies are no longer un-parented during conversion. For best performance and guaranteed up-to-date values, it is still recommended that static bodies either be root-level entities, or that static shapes be set up as compounds. Dynamic and kinematic bodies are still un-parented. If your game code assumes static bodies are converted into world space, you may need to decompose `LocalToWorld` using `Math.DecomposeRigidBodyTransform()` rather than reading directly from `Translation` and `Rotation`.

### Changes

* Dependencies
    * Updated Entities from `0.3.0-preview.4` to `0.6.0-preview.24`
    * Added Burst `1.3.0-preview.3`
    * Added Collections `0.5.2-preview.8`
    * Added Jobs `0.2.5-preview.20`
    * Added Mathematics `1.1.0`

* Run-Time API
    * Added the following new types:
        * `CollisionEvents` (allows iterating through events directly using a `foreach` loop, rather than only via `ICollisionEventsJob`)
        * `DispatchPairSequencer`
        * `IBodyPairsJobBase`
        * `ICollisionEventsJobBase`
        * `IContactsJobBase`
        * `IJacobiansJobBase`
        * `ITriggerEventsJobBase`
        * `Integrator`
        * `JointFrame`
        * `Math.FloatRange`
        * `NarrowPhase`
        * `SimulationContext`
        * `SimulationJobHandles`
        * `Solver`
        * `TriggerEvents` (allows iterating through events directly using a `foreach` loop, rather than only via `ITriggerEventsJob`)
        * `Velocity`
    * Added the following members:
        * `CollisionWorld.DynamicBodies`
        * `CollisionWorld.NumDynamicBodies`
        * `CollisionWorld.NumStaticBodies`
        * `CollisionWorld.StaticBodies`
        * `CollisionWorld.BuildBroadphase()`
        * `CollisionWorld.FindOverlaps()`
        * `CollisionWorld.Reset()`
        * `CollisionWorld.ScheduleBuildBroadphaseJobs()`
        * `CollisionWorld.ScheduleFindOverlapsJobs()`
        * `CollisionWorld.ScheduleUpdateDynamicTree()`
        * `CollisionWorld.UpdateDynamicTree()`
        * `DynamicsWorld.Reset()`
        * `Math.DecomposeRigidBodyOrientation()`
        * `Math.DecomposeRigidBodyTransform()`
        * `MassFactors.InverseInertiaFactorA`
        * `MassFactors.InverseInertiaFactorB`
        * `MassFactors.InverseMassFactorA`
        * `MassFactors.InverseMassFactorB`
        * `MotionVelocity.InverseInertia`
        * `MotionVelocity.InverseMass`
        * `Simulation.CollisionEvents`
        * `Simulation.TriggerEvents`
        * `Simulation.StepImmediate()`
    * Changed the following members/types:
        * `CollisionWorld` constructor now requires specifying the number of dynamic and static bodies separately
        * `CollisionWorld.NumBodies` is now read-only
        * `DynamicsWorld.NumJoints` is now read-only
        * `DynamicsWorld.NumMotions` is now read-only
        * `ISimulation.ScheduleStepJobs()` now requires callbacks and thread count hint
        * `ITreeOverlapCollector.AddPairs()` now has optional bool parameter to swap the bodies
        * `Joint.JointData` is now `BlobAssetReference<Joint>` instead of `Joint*`
        * `RigidBody.Collider` is now `BlobAssetReference<Collider>` instead of `Collider*`
        * The following types no longer implement `ICloneable`. Their `Clone()` methods now return instances of their respective types rather than `object`:
            * `CollisionWorld`
            * `DynamicsWorld`
            * `PhysicsWorld`
        * The following types now implement `IEquatable<T>`:
            * `Constraint`
    * Deprecated the following members:
        * `CollisionFilter.IsValid` (use its opposite, `CollisionFilter.IsEmpty` instead)
        * `CollisionWorld.ScheduleUpdateDynamicLayer()` (use `ScheduleUpdateDynamicTree()` instead)
        * `Constraint.StiffSpring()` (use `Constraint.LimitedDistance()` instead)
        * `Constraint` factory signatures passing `float` values for ranges (use new signatures taking `FloatRange` instead):
            * `Cone()`
            * `Cylindrical()`
            * `Planar()`
            * `Twist()`
        * `JointData.Create()` signature passing `MTransform` and `Constraint[]` (use new signature that takes `JointFrame` and `NativeArray<Constraint>` instead)
        * `JointData.CreateStiffSpring()` (use `JointData.CreateLimitedDistance()` instead)
        * `JointData` factory signatures passing `float3` and `quaternion` pairs for joint frames (use new signatures taking `JointFrame` instead):
            * `CreateFixed()`
            * `CreateHinge()`
            * `CreateLimitedHinge()`
            * `CreatePrismatic()`
            * `CreateRagdoll()`
        * `MassFactors.InvInertiaAndMassFactorA` (use `InverseInertiaFactorA` and/or `InverseMassFactorA` instead)
        * `MassFactors.InvInertiaAndMassFactorB` (use `InverseInertiaFactorB` and/or `InverseMassFactorB` instead)
        * `MotionVelocity.InverseInertiaAndMass` (use `InverseInertia` and/or `InverseMass` instead)
        * `RigidBody.HasCollider` (use `Collider.IsCreated` instead)
        * `SimplexSolver.Solve()` signature passing `PhysicsWorld` (use new signature that does not require one, as it is not used)
        * `SimulationStepInput.ThreadCountHint` (supply desired value directly to `ScheduleStepJobs()`)
    * Removed the following expired members:
        * `Broadphase.ScheduleBuildJobs()` signature that does not specify gravity
        * `CollisionWorld.ScheduleUpdateDynamicLayer()` signature that does not specify gravity
        * `JointData.NumConstraints`
        * `MeshCollider.Create()` signatures taking `NativeArray<int>`
        * `PhysicsWorld.CollisionTolerance`
        * `SimplexSolver.Solve()` signature taking `NativeArray<SurfaceConstraintInfo>`
        * `SimulationStepInput.Callbacks` (removed without expiration; pass callbacks to `ScheduleStepJobs()`)

* Authoring/Conversion API
    * Added the following new types:
        * `LegacyJointConversionSystem`
        * `CapsuleGeometryAuthoring`
    * Added the following members:
        * `PhysicsShapeAuthoring.GetCapsuleProperties()` signature returning `CapsuleGeometryAuthoring`
        * `PhysicsShapeAuthoring.SetCapsule()` signature passing `CapsuleGeometryAuthoring`
    * Deprecated the following members:
        * `PhysicsShapeAuthoring.GetCapsuleProperties()` signature returning `CapsuleGeometry`
        * `PhysicsShapeAuthoring.SetCapsule()` signature passing `CapsuleGeometry`
    * Removed the following expired members/types:
        * `BaseShapeConversionSystem<T>.ProduceColliderBlob()`
        * `DisplayCollisionEventsSystem.DisplayCollisionEventsJob`
        * `DisplayCollisionEventsSystem.FinishDisplayCollisionEventsJob`
        * `DisplayCollisionEventsSystem.ScheduleCollisionEventsJob()`
        * `DisplayContactsSystem.DisplayContactsJob`
        * `DisplayContactsSystem.FinishDisplayContactsJob`
        * `DisplayContactsSystem.ScheduleContactsJob()`
        * `DisplayTriggerEventsSystem.DisplayTriggerEventsJob`
        * `DisplayTriggerEventsSystem.FinishDisplayTriggerEventsJob`
        * `DisplayTriggerEventsSystem.ScheduleTriggerEventsJob()`
        * `PhysicsShapeAuthoring.GetMeshProperties()` using `NativeList<int>`. Use the signature taking `NativeList<int3>` instead

* Run-Time Behavior
    * The data protocol for static rigid bodies has changed so that at least one of `Translation`, `Rotation`, or `LocalToWorld` is required.
        * If a static body as a `Parent`, then its transform is always decomposed from its `LocalToWorld` component (which may be the result of last frame's transformations if you have not manually updated it).
        * If there is no `Parent`, then the body is assumed to be in world space and its `Translation` and `Rotation` are read directly as before, if they exist; otherwise corresponding values are decomposed from `LocalToWorld` if it exists.
        * In any case where a value must be decomposed from `LocalToWorld` but none exists, then a corresponding identity value is used (`float3.zero` for `Translation` and `quaternion.identity` for `Rotation`).
    * The following `CollisionWorld` queries no longer assert if the supplied collision filter is empty:
        * `CalculateDistance()`
        * `CastCollider()`
        * `CastRay()`
        * `OverlapAabb()`
    * `Simulation.ScheduleStepJobs()` can now schedule different types of simulation:
        * Providing a valid `threadCountHint` (>0) as the last parameter will schedule a multi-threaded simulation
        * Not providing a `threadCount` or providing an invalid value (<=0) will result in a simulation with a very small number of jobs being spawned (one per step phase)
        * Both simulation types offer the exact same customization options (callbacks) and outputs (events)
    * `ISimulation.Step()` is intended to allow you to step a simulation immediately in the calling code instead of spawning jobs
        * Unfortunately, `Unity.Physics.Simulation` is currently not Burst-compatible, and therefore can't be wrapped in a Burst compiled job
        * `Simulation.StepImmediate()` should be wrapped in a single Burst compiled job for lightweight stepping logic; however, it doesn't support callbacks
        * If callbacks are needed, one should implement the physics step using a set of function calls that represent different phases of the physics engine and add customization logic in between these calls; this should all be wrapped in a Burst compiled job; check `Simulation.StepImmediate()` for the full list of functions that need to be called
    * Previously, if an Entity referenced by a `PhysicsJoint` component was deleted, `BuildPhysicsWorld.CreateJoints()` would assert after failing to find a valid rigid body index. Now, if no valid body is found, the `Joint.BodyIndexPair` is marked as invalid instead and the simulation will ignore them.
    * Triggers no longer influence the center of mass or inertia of any compound colliders that they are part of.

* Authoring/Conversion Behavior
    * Implicitly static shapes (as well as explicitly static bodies) in a hierarchy are no longer un-parented during the conversion process. Dynamic and kinematic bodies still are.
    * Bevel radius and convex hull simplification tolerance are no longer modified by an object's scale when it is converted.
    * Improved performance of converting mesh colliders and convex hulls, particularly when they appear as multiple child instances in a compound collider.
    * Improved visualization of primitive shapes in the SceneView.
    * `PhysicsShapeAuthoring` components are now editable from within the SceneView.
    * `PhysicsShapeAuthoring` primitives are now oriented more correctly when non uniformly scaled.

### Fixes

* Fixed regression where displaying a mesh or convex hull preview on a shape authoring component immediately after a domain reload would trigger an assert.
* Fixed possible bug where classic `MeshCollider` components might convert to the incorrect shape.
* Frame Selected (F key in scene view) now properly focuses on the collision shape bounds.
* `ConvexCollider.Create()` had been allocating too much memory for `FaceVerticesIndex` array.
* `MotionVelocity.CalculateExpansion()` formula changed to reduce expansion of AABB due to rotation.
* `ForceUnique` checkbox in the Inspector is no longer disabled for primitive `PhysicsShape` types.
* Fixed a bug in scheduling resulting in race conditions during the solving phase.
* `PhysicsStepAuthoring` and `PhysicsDebugDisplayAuthoring` no longer throw exceptions when embedded in a sub-scene.

### Known Issues

* Due to a bug in the `TRSToLocalToParent` system, static bodies in a hierarchy will always cause the broad phase to be rebuilt. This will be fixed in the next version of entities after 0.6.0.
* Not all properties on classic joint components are converted yet:
    * Converted properties include connected bodies, axes, collisions, anchors, and linear/angular limits.
    * Motors, spring forces, mass scale, break limits, and projection settings are currently ignored.
    * Swapping bodies on `ConfigurableJoint` is not yet supported.
    * `ConfigurableJoint` setups with at least one free axis of angular motion are only stable if both other axes are locked (hinge), or both other axes are free (ball-and-socket).
    * Because solver behavior differs, some configurations also currently have a different feel from their classic counterparts.
* Modifying classic joints with Live Link enabled will currently leak memory.
* SceneView editing of `PhysicsShapeAuthoring` components currently only affects their size, but not their center offset.

## [0.2.5-preview.1] - 2019-12-05

### Fixes

* Fixed a bug that could cause compound colliders to not update with Live Link when moving around a mesh collider in the compound's hierarchy.
* Fixed possible incorrect instancing of colliders with different inputs

## [0.2.5-preview] - 2019-12-04

### Upgrade guide

* By default, `PhysicsColliders` that share the same set of inputs and that originate from the same sub-scene should reference the same data at run-time. This change not only reduces memory pressure, but also speeds up conversion. If you were altering `PhysicsCollider` data at run-time, you need to enable the `ForceUnique` setting on the respective `PhysicsColliderAuthoring` component. This setting guarantees the object will always convert into a unique instance.
* Any usages of `BlockStream` should be replaced with the `NativeStream` type from the com.unity.collections package.

### Changes

* Run-Time API
    * Removed `BlockStream` and migrated all usages to `NativeSteam`.
    * Access to `JointData.Constraints` has changed to an indexer. This means that to change the values of a `Constraint`, a copy should be made first. E.g.,
        ```c#
        var c = JointData.Constraints[index];
        c.Min = 0;
        JointData.Constraints[index] = c;
        ```
    * Added `maxVelocity` parameter to `SimplexSolver.Solve()` to clamp the solving results to the maximum value.
    * Added `SurfaceConstraintInfo.IsTooSteep` to indicate that a particular surface constraint has a slope bigger than the slope character controller supports.
    * `MeshCollider.Create()` now takes grouped triangle indices (NativeArray<int3>) instead of a flat list of indices (NativeArray<int>) as input.
    * Removed the following expired members/types:
        * `BoxCollider.ConvexRadius` (renamed to `BevelRadius`)
        * `CyllinderCollider.ConvexRadius` (renamed to `BevelRadius`)
        * Collider factory methods passing nullable types and numeric primitives:
            * `BoxCollider.Create()`
            * `CyllinderCollider.Create()`
            * `CapsuleCollider.Create()`
            * `ConvexCollider.Create()`
            * `MeshCollider.Create()`
            * `PolygonCollider.CreateQuad()`
            * `PolygonCollider.CreateTriangle()`
            * `SphereCollider.Create()`
            * `TerrainCollider.Create()`
        * `ComponentExtensions` members passing Entity (use components or variants passing component data instead):
            * `ApplyAngularImpulse()`
            * `ApplyImpulse()`
            * `ApplyLinearImpulse()`
            * `GetAngularVelocity()`
            * `GetCenterOfMass()`
            * `GetCollisionFilter()`
            * `GetEffectiveMass()`
            * `GetLinearVelocity()`
            * `GetMass()`
            * `GetPosition()`
            * `GetRotation()`
            * `GetVelocities()`
            * `SetAngularVelocity()`
            * `SetLinearVelocity()`
            * `SetVelocities()`
        * `CustomDataPair`
        * `ISimulation.ScheduleStepJobs()`
        * `JacobianFlags.EnableEnableMaxImpulse`
        * `MaterialFlags.EnableMaxImpulse`
        * `ModifiableContactHeader.BodyCustomDatas`
        * `ModifiableJacobianHeader.HasMaxImpulse`
        * `ModifiableJacobianHeader.MaxImpulse`
        * `ModifiableContactJacobian.CoefficientOfRestitution`
        * `ModifiableContactJacobian.FrictionEffectiveMassOffDiag`
        * `PhysicsCustomData`
        * `SimplexSolver.c_SimplexSolverEpsilon`
        * `SimplexSolver.Solve()` without `minDeltaTime` parameter

* Authoring/Conversion API
    * Added `PhysicsShapeAuthoring.ForceUnique`.
    * Added the following conversion systems:
        * `BeginColliderConversionSystem`
        * `BuildCompoundCollidersConversionSystem`
        * `EndColliderConversionSystem`
    * `PhysicsShapeAuthoring.GetMeshProperties()` now populates a `NativeList<int3>` for indices, instead of a `NativeList<int>`.
    * The following public members have been made protected:
        * `DisplayCollisionEventsSystem.FinishDisplayCollisionEventsJob`
        * `DisplayTriggerEventsSystem.FinishDisplayTriggerEventsJob`
    * Removed the following expired members/types:
        * `BaseShapeConversionSystem<T>.GetCustomFlags()`
        * `DisplayCollidersSystem.DrawComponent`
        * `DisplayCollidersSystem.DrawComponent.DisplayResult`
        * `DisplayContactsJob`
        * `DisplayJointsJob`
        * `FinishDisplayContactsJob`
        * `PhysicsShapeAuthoring` members:
            * `SetConvexHull()` passing only a mesh
            * `GetMesh()` replaced with `GetMeshProperties`
            * `ConvexRadius` replaced with `BevelRadius`
            * `GetBoxProperties()` returning `void`
            * `SetBox()` passing a box center, size and orientation. Pass `BoxGeometry` instead.
            * `GetCapsuleProperties()` returning `void`
            * `SetCapsule()` passing a capsule center, height, radius and orientation. Pass `CapsuleGeometry` instead.
            * `GetCylinderProperties()` returning `void`
            * `SetCylinder()` passing a cylinder center, height, radius and orientation. Pass `CylinderGeometry` instead.
            * `GetSphereProperties()` returning `void`
            * `SetSphere()` passing a sphere center, radius and orientation. Pass `SphereGeometry` instead.
        * The following components have been removed:
            * `PhysicsBody` (renamed to `PhysicsBodyAuthoring`)
            * `PhysicsShape` (renamed to `PhysicsShapeAuthoring`)
            * `PhysicsStep` (renamed to `PhysicsStepAuthoring`)
            * `PhysicsDebugDisplay` (renamed to `PhysicsDebugDisplayAuthoring`)

* Run-Time Behavior
    * `CompoundCollider.Create()` is now compatible with Burst.
    * `CompoundCollider.Create()` now correctly shares memory among repeated instances in the list of children.

* Authoring/Conversion Behavior
    * If mesh and convex `PhysicsShapeAuthoring` components have not explicitly opted in to `ForceUnique`, they may share the same `PhysicsCollider` data at run-time, if their inputs are the same.
    * Classic `MeshCollider` instances with the same inputs will always share the same data at run-time when converted in a sub-scene.
    * Further improved performance of collider conversion for all types.

### Fixes

* `JointData.Version` was not being incremented with changes to its properties.
* Fixed the issue of uninitialized array when scheduling collision event jobs with no dynamic bodies in the scene.
* Fixed the issue of `CollisionEvent.CalculateDetails()` reporting 0 contact points in some cases.
* Fixed the issue of joints with enabled collisions being solved after contacts for the same body pair.
* Fixed exception and leak caused by trying to display a convex hull preview for a physics shape with no render mesh assigned.
* Fixed bug causing incremental changes to a compound collider to accumulate ghost colliders when using Live Link.
* Fixed an issue where kinematic (i.e. infinite mass dynamic) bodies did not write to the collision event stream correctly.

### Known Issues

* Compound collider mass properties are not correctly updated while editing using Live Link.


## [0.2.4-preview] - 2019-09-19

### Changes

* Updated dependency on `com.unity.entities` to version `0.1.1-preview`. If you need to stay on entities version `0.0.12-preview.33` then you can use the previous version of this package, `0.2.3-preview`, which is feature equivalent:



## [0.2.3-preview] - 2019-09-19

### Upgrade guide

* Implicitly static shapes (i.e. those without a `PhysicsBodyAuthoring` or `Rigidbody`) in a hierarchy under a GameObject with `StaticOptimizeEntity` are now converted into a single compound `PhysicsCollider` on the entity with the `Static` tag. If your queries or contact events need to know about data associated with the entities from which these leaf shapes were created, you need to explicitly add `PhysicsBodyAuthoring` components with static motion type, in order to prevent them from becoming part of the compound.

### Changes

* Run-Time API
    * Deprecated `Broadphase.ScheduleBuildJobs()` and provided a new implementation that takes gravity as input.
    * Deprecated `CollisionWorld.ScheduleUpdateDynamicLayer()` and provided a new implementation that takes gravity as input.
    * Deprecated `PhysicsWorld.CollisionTolerance` and moved it to `CollisionWorld`.
    * Deprecated `ManifoldQueries.BodyBody()` and provided a new implementation that takes two bodies and two velocities.
    * Added `CollisionEvent.CalculateDetails()` which provides extra information about the collision event:
        * Estimated impulse
        * Estimated impact position
        * Array of contact point positions
    * Removed `CollisionEvent.AccumulatedImpulses` and provided `CollisionEvent.CalculateDetails()` which gives a more reliable impulse value.

* Authoring/Conversion API
    * Removed the following expired ScriptableObject:
        * `CustomFlagNames`
    * Removed the following expired members:
        * `PhysicsShapeAuthoring.GetBelongsTo()`
        * `PhysicsShapeAuthoring.SetBelongsTo()`
        * `PhysicsShapeAuthoring.GetCollidesWith()`
        * `PhysicsShapeAuthoring.SetCollidesWith()`
        * `PhysicsShapeAuthoring.OverrideCustomFlags`
        * `PhysicsShapeAuthoring.CustomFlags`
        * `PhysicsShapeAuthoring.GetCustomFlag()`
        * `PhysicsShapeAuthoring.SetCustomFlag()`
        * `PhysicsMaterialTemplate.GetBelongsTo()`
        * `PhysicsMaterialTemplate.SetBelongsTo()`
        * `PhysicsMaterialTemplate.GetCollidesWith()`
        * `PhysicsMaterialTemplate.SetCollidesWith()`
        * `PhysicsMaterialTemplate.CustomFlags`
        * `PhysicsMaterialTemplate.GetCustomFlag()`
        * `PhysicsMaterialTemplate.SetCustomFlag()`

* Run-Time Behavior
    * Gravity is now applied at the beginning of the step, as opposed to previously being applied at the end during integration.

* Authoring/Conversion Behavior
    * Implicitly static shapes in a hierarchy under a GameObject with `StaticOptimizeEntity` are now converted into a single compound `PhysicsCollider`.

### Fixes

* Fixed issues preventing compatibility with DOTS Runtime.
* Fixed occasional tunneling of boxes through other boxes and mesh triangles.
* Fixed incorrect AABB sweep direction during collisions with composite colliders, potentially allowing tunneling.
* Fixed obsolete `MeshCollider.Create()` creating an empty mesh.
* Fixed obsolete `ConvexCollider.Create()` resulting in infinite recursion.
* Fixed simplex solver bug causing too high output velocities in 3D solve case.
* Fixed bug causing custom meshes to be ignored on convex shapes when the shape's transform was bound to a skinned mesh.
* Fixed Burst incompatibilities in the following types:
    * `BoxGeometry`
    * `CapsuleGeometry`
    * `CollisionFilter`
    * `ConvexHullGenerationParameters`
    * `CylinderGeometry`
    * `Material`
    * `SphereGeometry`
* Improved performance of `MeshConnectivityBuilder.WeldVertices()`.
* Fixed a potential assert when joints are created with a static and dynamic body (in that order).

### Known Issues



## [0.2.2-preview] - 2019-09-06

### Fixes

* Added internal API extensions to work around an API updater issue with Unity 2019.1 to provide a better upgrading experience.



## [0.2.1-preview] - 2019-09-06

### Upgrade guide

* A few changes have been made to convex hulls that require double checking convex `PhysicsShapeAuthoring` components:
    * Default parameters for generating convex hulls have been tweaked, which could result in minor differences.
    * Bevel radius now applies a shrink to the shape rather than an expansion (as with primitive shape types).
* Mesh `PhysicsShapeAuthoring` objects with no custom mesh assigned now include points from enabled mesh renderers on their children (like convex shapes). Double check any mesh shapes in your projects.
* Due to a bug in version 0.2.0, any box colliders added to uniformly scaled objects had their scale baked into the box size parameter when initially added and/or when fit to render geometry. Double check box colliders on any uniformly scaled objects and update them as needed (usually by just re-fitting them to the render geometry).
* The serialization layout of `PhysicsShapeAuthoring` has changed. Values previously saved in the `m_ConvexRadius` field will be migrated to `m_ConvexHullGenerationParameters.m_BevelRadius`, and a `m_ConvexRadius_Deprecated` field will then store a negative value to indicate the old data have been migrated. Because this happens automatically when objects are deserialized, prefab instances may mark this field dirty even if the prefab has already been migrated. Double check prefab overrides for Bevel Radius on your prefab instances.

### Changes

* Run-Time API
    * Added the following new members:
        * `BodyIndexPair.IsValid`
        * `Math.Dotxyz1()` using double
        * `Plane.Projection()`
        * `Plane.SignedDistanceToPoint()`
    * Added the following structs:
        * `BoxGeometry`
        * `CapsuleGeometry`
        * `CylinderGeometry`
        * `SphereGeometry`
        * `ConvexHullGenerationParameters`
    * `Constraint` now implements `IEquatable<Constraint>` to avoid boxing allocations.
    * All previous `SphereCollider`, `CapsuleCollider`, `BoxCollider` and `CylinderCollider` properties are now read only. A new `Geometry` property allows reading or writing all the properties at once.
    * `BoxCollider.Create()` now uses `BoxGeometry`. The signature passing nullable types has been deprecated.
    * `CapsuleCollider.Create()` now uses `CapsuleGeometry`. The signature passing nullable types has been deprecated.
    * `ConvexCollider.Create()` now uses `ConvexHullGenerationParameters`. The signature passing nullable types has been deprecated.
    * `CylinderCollider.Create()` now uses `CylinderGeometry`. The signature passing nullable types has been deprecated.
    * `MeshCollider.Create()` now uses native containers. The signature using managed containers has been deprecated.
    * `PolygonCollider.CreateQuad()` signature passing nullable types has been deprecated.
    * `PolygonCollider.CreateTriangle()` signature passing nullable types has been deprecated.
    * `SphereCollider.Create()` now uses `SphereGeometry`. The signature passing nullable types has been deprecated.
    * `TerrainCollider.Create()` signature passing pointer and nullable types has been deprecated.
    * `SimplexSolver.Solve()` taking the `respectMinDeltaTime` has been deprecated. Use the new `SimplexSolver.Solve()` method that takes `minDeltaTime` instead.
    * Renamed `BoxCollider.ConvexRadius` to `BevelRadius`.
    * Renamed `CylinderCollider.ConvexRadius` to `BevelRadius`.
    * Deprecated `SimplexSolver.c_SimplexSolverEpsilon`.
    * Deprecated the following methods in `ComponentExtensions` taking an `Entity` as the first argument.
        * `GetCollisionFilter()`
        * `GetMass()`
        * `GetEffectiveMass()`
        * `GetCenterOfMass()`
        * `GetPosition()`
        * `GetRotation()`
        * `GetVelocities()`
        * `SetVelocities()`
        * `GetLinearVelocity()`
        * `SetLinearVelocity()`
        * `GetAngularVelocity()`
        * `SetAngularVelocity()`
        * `ApplyImpulse()`
        * `ApplyLinearImpulse()`
        * `ApplyAngularImpulse()`
    * Removed the following expired members:
        * `ColliderCastInput.Direction`
        * `ColliderCastInput.Position`
        * `Ray(float3, float3)`
        * `Ray.Direction`
        * `Ray.ReciprocalDirection`
        * `RaycastInput.Direction`
        * `RaycastInput.Position`
* Authoring/Conversion API
    * Renamed `PhysicsBody` to `PhysicsBodyAuthoring`.
    * Renamed `PhysicsShape` to `PhysicsShapeAuthoring`.
    * `PhysicsShapeAuthoring.BevelRadius` now returns the serialized bevel radius data in all cases, instead of returning the shape radius when the type was either sphere or capsule, or a value of 0 for meshes. Its setter now only clamps the value if the shape type is box or cylinder.
    * `PhysicsShapeAuthoring.GetBoxProperties()` now returns `BoxGeometry`. The signature containing out parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetBox()` now uses `BoxGeometry`. The signature passing individual parameters has been deprecated.
    * `PhysicsShapeAuthoring.GetCapsuleProperties()` now returns `CapsuleGeometry`. The signature containing out parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetCapsule()` now uses `CapsuleGeometry`. The signature passing individual parameters has been deprecated.
    * `PhysicsShapeAuthoring.GetCylinderGeometry()` now returns `CylinderGeometry`. The signature containing out parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetCylinder()` now uses `CylinderGeometry`. The signature passing individual parameters has been deprecated.
    * `PhysicsShapeAuthoring.GetSphereProperties()` now returns `SphereGeometry`. The signature containing out parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetSphere()` now uses `SphereGeometry`. The signature passing individual parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetConvexHull()` now uses `ConvexHullGenerationParameters`. The old signature has been deprecated.
    * `PhysicsShapeAuthoring.ConvexRadius` has been deprecated. Instead use `BevelRadius` values returned by geometry structs.
    * `PhysicsShapeAuthoring.GetConvexHullProperties()` now returns points from `SkinnedMeshRenderer` components that are bound to the shape's transform, or to the transforms of its children that are not children of some other shape, when no custom mesh has been assigned.
    * Added `PhysicsShapeAuthoring.SetConvexHull()` signature specifying minimum skinned vertex weight for inclusion.
    * Added `PhysicsShapeAuthoring.ConvexHullGenerationParameters` property.
    * `PhysicsShapeAuthoring.GetMesh()` has been deprecated.
    * Added `PhysicsShapeAuthoring.GetMeshProperties()`. When no custom mesh has been assigned, this will return mesh data from all render geometry in the shape's hierarchy, that are not children of some other shape.
    * `PhysicsShapeAuthoring.FitToEnabledRenderMeshes()` now takes an optional parameter for specifying minimum skinned vertex weight for inclusion.
    * Removed the `OutputStream` field from various deprecated debug drawing jobs. These will be redesigned in a future release, and you are advised to not try to extend them yet.
    * Removed the following expired members:
        * `PhysicsShapeAuthoring.FitToGeometry()`.
        * `PhysicsShapeAuthoring.GetCapsuleProperties()` returning raw points.
        * `PhysicsShapeAuthoring.GetPlaneProperties()` returning raw points.
        * `FirstPassLegacyRigidbodyConversionSystem`.
        * `FirstPassPhysicsBodyConversionSystem`.
        * `SecondPassLegacyRigidbodyConversionSystem`.
        * `SecondPassPhysicsBodyConversionSystem`.
* Run-Time Behavior
    * `BoxCollider.Create()` is now compatible with Burst.
    * `CapsuleCollider.Create()` is now compatible with Burst.
    * `ConvexCollider.Create()` is now compatible with Burst.
    * `CylinderCollider.Create()` is now compatible with Burst.
    * `MeshCollider.Create()` is now compatible with Burst.
    * `SphereCollider.Create()` is now compatible with Burst.
    * `TerrainCollider.Create()` is now compatible with Burst.
* Authoring/Conversion Behavior
    * Converting mesh and convex shapes is now several orders of magnitude faster.
    * Convex meshes are more accurate and less prone to jitter.
    * `PhysicsShapeAuthoring` components set to convex now display a wire frame preview at edit time.
    * `PhysicsShapeAuthoring` components set to cylinder can now specify how many sides the generated hull should have.
    * Inspector controls for physics categories, custom material tags, and custom body tags now have a final option to select and edit the corresponding naming asset.

### Fixes

* Body hierarchies with multiple shape types (e.g., classic collider types and `PhysicsShapeAuthoring`) now produce a single flat `CompoundCollider` tree, instead of a tree with several `CompoundCollider` leaves.
* Fixed issues causing dynamic objects to tunnel through thin static objects (most likely meshes)
* Fixed incorrect behavior of Constraint.Twist() with limitedAxis != 0
* Fixed regression introduced in 0.2.0 causing box shapes on uniformly scaled objects to always convert into a box with size 1 on all sides.
* Fixed exception when calling `Dispose()` on an uninitialized `CollisionWorld`.

### Known Issues

* Wire frame previews for convex `PhysicsShapeAuthoring` components can take a while to generate.
* Wire frame previews for convex `PhysicsShapeAuthoring` components do not currently illustrate effects of bevel radius in the same way as primitives.
* The first time you convert convex or mesh shapes in the Editor after a domain reload, you will notice a delay while the conversion jobs are Burst compiled. all subsequent conversions should be significantly faster until the next domain reload.
* Updated dependency on `com.unity.burst` to version `1.1.2`.



## [0.2.0-preview] - 2019-07-18

### Upgrade guide

* If you created per-body custom data using `PhysicsShape.CustomFlags` then you should instead do it using `PhysicsBody.CustomTags`.
* Some public API points were _removed_, either because they were not intended to be public or because they introduce other problems (see Changes below).
    * Some of these API points may later be reintroduced on a case-by-case basis to enable customization for advanced use cases. Please provide feedback on the forums if these removals have affected current use cases so we can prioritize them.
* Some public API points were _replaced_ with another one in a way that cannot be handled by the script updater, so they must be manually fixed in your own code (see Changes below).
* All public types in test assemblies were never intended to be public and have been made internal.
* Some properties on `PhysicsMaterialTemplate` and `PhysicsShape` now return `PhysicsCategoryTags` instead of `int`. Use its `Value` property if you need to get/set a raw `int` value (see Changes below).

### Changes

* Run-Time API
    * Added first draft of a new `TerrainCollider` struct. Terrain geometry is defined by a height field. It requires less memory than an equivalent mesh and is faster to query. It also offers a fast, low-quality option for collision detection.
        * Added `ColliderType.Terrain`.
        * Added `CollisionType.Terrain`.
        * Added `DistanceQueries.ConvexTerrain<T>()`.
        * Added `DistanceQueries.PointTerrain<T>()`.
    * Shapes and bodies how have their own separate custom data.
        * Added `Material.CustomTags` (for shapes).
        * Replaced `PhysicsCustomData` with `PhysicsCustomTags` (for bodies).
        * Replaced `ContactHeader.BodyCustomDatas` with `ContactHeader.BodyCustomTags`.
        * Replaced `CustomDataPair` with `CustomTagsPair`.
        * Replaced `RigidBody.CustomData` with `RigidBody.CustomTags`.
    * Removed coefficient of restitution concept from Jacobians. All restitution calculations are approximated and applied before the solve, so restitution changes at this point in the simulation have no effect.
        * `ModifiableContactJacobian.CoefficientOfRestitution` is now obsolete.
    * `ModifiableContactJacobian.FrictionEffectiveMassOffDiag` is now obsolete. It was not possible to make any meaningful changes to it without fully understanding friction solving internals.
    * Removed max impulse concept from Jacobians. Solver design implies impulses are pretty unpredictable, making it difficult to choose maximum impulse value in practice.
        * `JacobianFlags.EnableMaxImpulse` is now obsolete.
            * Underlying values of `JacobianFlags` have been changed.
            * Added `JacobianFlags.UserFlag2`.
        * `Material.EnableMaxImpulse` is now obsolete.
        * `MaterialFlags.EnableMaxImpulse` is now obsolete.
        * `ModifiableJacobianHeader.HasMaxImpulse` is now obsolete.
        * `ModifiableJacobianHeader.MaxImpulse` is now obsolete.
    * Removed the following members:
        * `CollisionFilter.CategoryBits`
        * `CollisionFilter.MaskBits`
    * Removed the following types from public API and made them internal:
        * `AngularLimit1DJacobian`
        * `AngularLimit2DJacobian`
        * `AngularLimit3DJacobian`
        * `BaseContactJacobian`
        * `BoundingVolumeHierarchy.BuildBranchesJob`
        * `BoundingVolumeHierarchy.BuildFirstNLevelsJob`
        * `BoundingVolumeHierarchy.FinalizeTreeJob`
        * `Broadphase`
        * `ColliderCastQueries`
        * `CollisionEvent`
        * `CollisionEvents`
        * `ContactHeader`
        * `ContactJacobian`
        * `ConvexConvexDistanceQueries`
        * `ConvexHull`
        * `ConvexHullBuilder`
        * `ConvexHullBuilderExtensions`
        * `DistanceQueries`
        * `ElementPool<T>` (Unity.Collections)
        * `Integrator`
        * `IPoolElement` (Unity.Collections)
        * `JacobianHeader`
        * `JacobianIterator`
        * `JacobianUtilities`
        * `LinearLimitJacobian`
        * `ManifoldQueries`
        * `Mesh`
        * `MotionExpansion`
        * `NarrowPhase`
        * `OverlapQueries`
        * `QueryWrappers`
        * `RaycastQueries`
        * `Scheduler`
        * `Simulation.Context`
        * `Solver`
        * `StaticLayerChangeInfo`
        * `TriggerEvent`
        * `TriggerEvents`
        * `TriggerJacobian`
    * Removed the following members from public API and made them internal:
        * Aabb.CreateFromPoints(float3x4)
        * `BoundingVolumeHierarchy.BuildBranch()`
        * `BoundingVolumeHierarchy.BuildCombinedCollisionFilter()`
        * `BoundingVolumeHierarchy.BuildFirstNLevels()`
        * `BoundingVolumeHierarchy.CheckIntegrity()`
        * All explicit `ChildCollider` constructors
        * `ColliderKeyPath(ColliderKey, uint)`
        * `ColliderKeyPath.Empty`
        * `ColliderKeyPath.GetLeafKey()`
        * `ColliderKeyPath.PopChildKey()`
        * `ColliderKeyPath.PushChildKey()`
        * `CollisionWorld.Broadphase`
        * `CompoundCollider.BoundingVolumeHierarchy`
        * `Constraint.ConstrainedAxis1D`
        * `Constraint.Dimension`
        * `Constraint.FreeAxis2D`
        * `ConvexCollider.ConvexHull`
        * `MeshCollider.Mesh`
        * `MotionVelocity.CalculateExpansion()`
        * `SimulationCallbacks.Any()`
        * `SimulationCallbacks.Execute()`
    * `ChildCollider.TransformFromChild` is now a readonly property instead of a field.
    * Removed `BuildPhysicsWorld.m_StaticLayerChangeInfo`.
    * Added `FourTranposedAabbs.DistanceFromPointSquared()` signatures passing scale parameter.
    * Changed `ISimulation` interface (i.e. `Simulation` class).
        * Added `ISimulation.FinalJobJandle`.
        * Added `ISimulation.FinalSimulationJobJandle`.
        * Added simpler `ISimulation.ScheduleStepJobs()` signature and marked previous one obsolete.
    * Added `RigidBody.HasCollider`.
    * `SimplexSolver.Solve()` now takes an optional bool to specify whether it should respect minimum delta time.
    * `SurfaceVelocity.ExtraFrictionDv` has been removed and replaced with more usable `LinearVelocity` and `AngularVelocity` members.
* Authoring/Conversion API
    * Added `CustomBodyTagNames`.
    * Renamed `CustomMaterialTagNames.FlagNames` to `CustomMaterialTagNames.TagNames`.
    * Renamed `CustomFlagNames` to `CustomPhysicsMaterialTagNames`.
    * Renamed `CustomPhysicsMaterialTagNames.FlagNames` to `CustomPhysicsMaterialTagNames.TagNames`.
    * Added `PhysicsCategoryTags`, `CustomBodyTags`, and `CustomMaterialTags` authoring structs.
    * The following properties now return `PhysicsCategoryTags` instead of `int`:
        * `PhysicsMaterialTemplate.BelongsTo`
        * `PhysicsMaterialTemplate.CollidesWith`
        * `PhysicsShape.BelongsTo`
        * `PhysicsShape.CollidesWith`
    * The following members on `PhysicsMaterialTemplate` are now obsolete:
        * `GetBelongsTo()`
        * `SetBelongsTo()`
        * `GetCollidesWith()`
        * `SetCollidesWith()`
    * The following members on `PhysicsShape` are now obsolete:
        * `GetBelongsTo()`
        * `SetBelongsTo()`
        * `GetCollidesWith()`
        * `SetCollidesWith()`
    * Added `PhysicsMaterialTemplate.CustomTags`.
        * `CustomFlags`, `GetCustomFlag()` and `SetCustomFlag()` are now obsolete.
    * Added `PhysicsShape.CustomTags`.
        * `CustomFlags`, `GetCustomFlag()` and `SetCustomFlag()` are now obsolete.
    * Added `PhysicsBody.CustomTags`.
    * Renamed `PhysicsShape.OverrideCustomFlags` to `PhysicsShape.OverrideCustomTags`.
    * Renamed `PhysicsShape.CustomFlags` to `PhysicsShape.CustomTags`.
    * Renamed `PhysicsShape.GetCustomFlag()` to `PhysicsShape.GetCustomTag()`.
    * Renamed `PhysicsShape.SetCustomFlag()` to `PhysicsShape.SetCustomTag()`.
    * Overload of `PhysicsShape.GetCapsuleProperties()` is now obsolete.
    * Overload of `PhysicsShape.GetPlaneProperties()` is now obsolete.
    * Removed `PhysicsBody.m_OverrideDefaultMassDistribution` (backing field never intended to be public).
    * `PhysicsShape.GetBoxProperties()` now returns underlying serialized data instead of reorienting/resizing when aligned to local axes.
    * `BaseShapeConversionSystem.GetCustomFlags()` is now obsolete.
    * Parameterless constructors have been made private for the following types because they should not be used (use instead ScriptableObjectCreateInstance<T>() or GameObject.AddComponent<T>() as applicable):
        * `CustomPhysicsMaterialTagNames`
        * `PhysicsCategoryNames`
        * `PhysicsMaterialTemplate`
        * `PhysicsBody`
        * `PhysicsShape`
    * The following types have been made sealed:
        * `LegacyBoxColliderConversionSystem`
        * `LegacyCapsuleColliderConversionSystem`
        * `LegacySphereColliderConversionSystem`
        * `LegacyMeshColliderConversionSystem`
        * `LegacyRigidbodyConversionSystem`
        * `PhysicsBodyConversionSystem`
        * `PhysicsShapeConversionSystem`
    * `DisplayContactsJob` has been deprecated in favor of protected `DisplayContactsSystem.DisplayContactsJob`.
    * `FinishDisplayContactsJob` has been deprecated in favor of protected `DisplayContactsSystem.FinishDisplayContactsJob`.
    * `DisplayJointsJob` has been deprecated in favor of protected `DisplayJointsSystem.DisplayJointsJob`.
    * `FinishDisplayTriggerEventsJob` has been deprecated in favor of protected `DisplayTriggerEventsSystem.FinishDisplayTriggerEventsJob`.
    * The following types have been deprecated and will be made internal in a future release:
        * `DisplayBodyColliders.DrawComponent`
        * `DisplayCollisionEventsSystem.FinishDisplayCollisionEventsJob`
* Run-Time Behavior
    * Collision events between infinite mass bodies (kinematic-kinematic and kinematic-static) are now raised. The reported impulse will be 0.
    * The default value of `Unity.Physics.PhysicsStep.ThreadCountHint` has been increased from 4 to 8.
    * `EndFramePhysicsSystem` no longer waits for `HandlesToWaitFor`, instead it produces a `FinalJobHandle` which is a combination of those jobs plus the built-in physics jobs. Subsequent systems that depend on all physics jobs being complete can use that as an input dependency.
* Authoring/Conversion Behavior
    * `PhysicsCustomData` is now converted from `PhysicsBody.CustomTags` instead of using the flags common to all leaf shapes.
    * `PhysicsShape.CustomTags` is now converted into `Material.CustomTags`.
    * If a shape conversion system throws an exception when producing a `PhysicsCollider`, then it is simply skipped and logs an error message, instead of causing the entire conversion process to fail.
    * `PhysicsShape` Inspector now displays suggestions of alternative primitive shape types if a simpler shape would yield the same collision result as the current configuration.
    * `PhysicsShape` Inspector now displays error messages if a custom mesh or discovered mesh is not compatible with run-time conversion.

### Fixes

* Draw Collider Edges now supports spheres, capsules, cylinders and compound colliders.
* Fixed bug causing editor to get caught in infinite loop when adding `PhysicsShape` component to a GameObject with deactivated children with `MeshFilter` components.
* Fixed bug resulting in the creation of `PhysicMaterial` objects in sub-scenes when converting legacy colliders.
* Fixed bug when scaling down friction on bounce in Jacobian building. Once a body was declared to bounce (apply restitution), all subsequent body Jacobians had their friction scaled down to 25%.
* Fixed bug resulting in the broadphase for static bodies possibly not being updated when adding or moving a static body, causing queries and collisions to miss.
* Fixed bug with restitution impulse during penetration recovery being too big due to wrong units used.
* Fixed bug with energy gain coming from restitution impulse with high restitution values.
* Fixed Jacobian structures being allocated at non 4 byte aligned addresses, which caused crashes on Android
* Removed Solver & Scheduler asserts from Joints between two static bodies #383.
* Fixed bug preventing the conversion of classic `BoxCollider` components with small sizes.
* Fixed bug where `PhysicsShape.ConvexRadius` setter was clamping already serialized value instead of newly assigned value.
* Fixed bug where `PhysicsShape` orientation, size, and convex radius values might undergo changes during conversion resulting in identical volumes; only objects inheriting non-uniform scale now exhibit this behavior.
* Fixed bug causing minor drawing error for cylinder `PhysicsShape` with non-zero convex radius.
* Fixed crash when trying to run-time convert a `MeshCollider` or `PhysicsShape` with a non-readable mesh assigned. Conversion system now logs an exception instead.
* Fixed crash when constructing a `MeshCollider` with no input triangles. A valid (empty) mesh is still created.
* Fixed bugs building to IL2CPP resulting in unresolved external symbols in `BvhLeafProcessor`, `ConvexCompoundLeafProcessor`, and `ConvexMeshLeafProcessor`.
* Fixed bug causing physics IJob's to not be burst compiled (ICollisionEventsJob, ITriggerEventsJob, IBodyPairsJob, IContactsJob, IJacobiansJob)



## [0.1.0-preview] - 2019-05-31

### Upgrade guide

* Any run-time code that traversed the transform hierarchy of physics objects to find other entities must instead store references to entity IDs of interest through a different mechanism.
* Baking of non-uniform scale for parametric `PhysicsShape` types has changed to more predictably approximate skewed render meshes. Double check the results on any non-uniformly scaled objects in your projects.
* Angular Velocity is currently set in Motion Space. The Motion Space orientation of dynamic bodies may have changed with the changes to the conversion pipeline. If dynamic bodies are now rotating differently, check the values of `Initial Angular Velocity` in the `PhysicsBody` component, or values set to `Angular` in the `PhysicsVelocity` component.
* Convex `PhysicsShape` objects with no custom mesh assigned now include points from enabled mesh renderers on their children. Double check any convex objects in your projects.
* The `RaycastInput` and `ColliderCastInput` structs have changed. See below for details.

### Changes

* Run-Time API
    * Renamed `CollisionFilter.CategoryBits` to `CollisionFilter.BelongsTo`.
    * Renamed `CollisionFilter.MaskBits` to `CollisionFilter.CollidesWith`.
    * `RaycastInput` and `ColliderCastInput` have changed:
        * At the input level, start and end positions are now specified rather then an initial position and a displacement.
        * `Start` replaces `Position`.
        * `End` replaces `Direction` which had been confusing as it was actually a displacement vector to a second point on the ray.
        * `Ray` has been made internal as a lower level interface and updated to be less ambiguous.
    * Added job interfaces for easier reading of simulation events, instead of having to work directly with block streams.
        * `ICollisionEventsJob` calls `Execute()` for every collision event produced by the solver.
        * `ITriggerEventsJob` calls `Execute()` for every trigger event produced by the solver.
        * These events now also include the Entity's involved, not just the rigid body indices.
    * Added job interfaces for easier modification of simulation data, instead of having to work directly with block streams.
        * `IBodyPairsJob` calls `Execute()` for every body pair produced by the broad phase.
        * `IContactsJob` calls `Execute()` for every contact manifold produced by the narrowphase.
        * `IJacobiansJob` calls `Execute()` for every contact jacobian before it is solved.
* Authoring/Conversion API
    * Renamed `SecondPassLegacyRigidbodyConversionSystem` to `LegacyRigidbodyConversionSystem`.
    * Deprecated `FirstPassPhysicsBodyConversionSystem`.
    * Renamed `SecondPassPhysicsBodyConversionSystem` to `PhysicsBodyConversionSystem`.
    * Deprecated `FirstPassLegacyRigidbodyConversionSystem`.
    * Deprecated overload of `PhysicsShape.GetCapsuleProperties()` returning raw points.
    * Deprecated overload of `PhysicsShape.GetPlaneProperties()` returning raw points.
    * Renamed `PhysicsShape.FitToGeometry()` to `PhysicsShape.FitToEnabledRenderMeshes()`. New method accounts for enabled `MeshRenderer` components on child objects with the same physics body and leaf shape.
    * `PhysicsShape.GetConvexHullProperties()` now includes points from enabled `MeshRenderer` components on child objects with the same physics body and leaf shape when no custom mesh has been assigned.
* Run-Time Behavior
    * Added a fixed angle constraint to prismatic joints so they no longer rotate freely.
    * Any Entity with a `PhysicsVelocity` component will be added to the `PhysicsWorld` and integrated, even if it has no `PhysicsCollider`.
* Authoring/Conversion Behavior
    * Physics data on deactivated objects in the hierarchy are no longer converted.
    * All physics objects (i.e. bodies) are now un-parented (required since all simulation happens in world space).
    * Legacy `Collider` components are no longer converted if they are disabled.
    * Converted `PhysicsShape` objects with non-uniform scale now bake more predictable results when their `Transform` is skewed.
    * All menu paths for custom assets and authoring components have changed from 'DOTS Physics' to 'DOTS/Physics'.

### Fixes

* Fixed trigger events not being raised between kinematic-vs-kinematic or kinematic-vs-static body pairs.
* Fixed crash in BuildPhysicsWorld when creating a dynamic body without a `PhysicsMass` component
* Cylinder/sphere GameObjects no longer appear in first frame when draw colliders is enabled on `PhysicsDebugDisplay`.
* Fix bugs in `BoundingVolumeHierarchy.Build` which now produces `BoundingVolumeHierarchy` of greater quality. This will affect performance of BVH queries, and most notably it will significantly improve performance of `DynamicVsDynamicFindOverlappingPairsJob`
* Fixed incorrect 3DOF angular constraint solving with non-identity joint orientation
* Fixed bug where converted `SphereCollider` would apply incorrect center offset when in a hierarchy with non-uniform scale.
* Fixed bug where converted `PhysicsShape` would not become part of a compound collider at run-time if the first `PhysicsBody` ancestor found higher up the hierarchy was disabled.
* Fixed bug where leaves of compound shapes in a hierarchy might be added to the wrong entity if it had disabled `UnityEngine.Collider` components.
* Fixed bug causing leaves of compound shapes on prefabs to convert into individual static colliders.
* Fixed restitution response to be more bouncy for convex objects with high restitution values
* Fixed bug where a converted GameObject in a hierarchy would have the wrong translation and rotation at run-time.
* Fixed bug where objects with `StaticOptimizeEntity` would not be converted into physics world.
* Preview for Mesh `PhysicsShape` no longer culls back faces.
* Inspector control for convex radius on `PhysisShape` now appears when shape type is convex hull.

### Known Issues

* Attempting to fit a non-uniformly scaled `PhysicsShape` to its render meshes may produce unexpected results.
* Physics objects loaded from sub-scenes may have the wrong transformations applied on Android.
* Dragging a control label to modify the orientation of a `PhysicsShape` sometimes produces small changes in its size.
* If gizmos are enabled in the Game tab, the 'Physics Debug Display' viewers are incorrectly rendered. Debug viewers render correctly in the Scene tab.



## [0.0.2-preview] - 2019-04-08

### Upgrade guide

* Any assembly definitions referencing `Unity.Physics.Authoring` assembly by name must be updated to instead reference `Unity.Physics.Hybrid`.

### Changes

* Renamed `Unity.Physics.Authoring` assembly to `Unity.Physics.Hybrid`. (All of its types still exist in the `Unity.Physics.Authoring` namespace.)
* Radius of cylinder `PhysicsShape` is no longer non-uniformly scaled when converted.

### Fixes

* Fixed exception when converting a box `PhysicsShape` with negative scale.
* Fixed incorrect orientation when fitting capsule, cylinder, or plane `PhysicsShape` to render mesh.
* Fixed convex radius being too large when switching `PhysicsShape` from plane to box or cylinder.
* Fixed calculation of minimum half-angle between faces in convex hulls.
* Fixed collision/trigger event iterators skipping some events when iterating.
* Fixed memory leak when enabling "Draw colliders" in the Physics Debug Display.

### Known Issues

* Physics systems are tied to (variable) rendering frame rate when using automatic world bootstrapping. See `FixedTimestep` examples in ECS Samples project for currently available approaches.
* IL2CPP player targets have not yet been fully validated.
* Some tests might occasionally fail due to JobTempAlloc memory leak warnings.
* Swapping `PhysicsShape` component between different shape types may produce non-intuitive results when nested in hierarchies with non-uniform scale.
* Some `PhysicsShape` configurations do not bake properly when nested in hierarchies with non-uniform scale.
* `PhysicsShape` does not yet visualize convex hull shapes at edit-time.
* Drag values on classic `Rigidbody` components are not currently converted correctly.



## [0.0.1-preview] - 2019-03-12

* Initial package version
