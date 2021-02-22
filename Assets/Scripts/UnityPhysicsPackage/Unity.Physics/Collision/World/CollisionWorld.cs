using System;
using Unity.Assertions;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using UnityS.Mathematics;
using static UnityS.Physics.Math;
using Unity.Jobs.LowLevel.Unsafe;

namespace UnityS.Physics
{
    // A collection of rigid bodies wrapped by a bounding volume hierarchy.
    // This allows to do collision queries such as raycasting, overlap testing, etc.
    [NoAlias]
    public struct CollisionWorld : ICollidable, IDisposable
    {
        [NoAlias] private NativeArray<RigidBody> m_Bodies;    // storage for all the rigid bodies
        [NoAlias] internal Broadphase Broadphase;             // bounding volume hierarchies around subsets of the rigid bodies
        [NoAlias] internal NativeHashMap<Entity, int> EntityBodyIndexMap;

        public int NumBodies => Broadphase.NumStaticBodies + Broadphase.NumDynamicBodies;
        public int NumStaticBodies => Broadphase.NumStaticBodies;
        public int NumDynamicBodies => Broadphase.NumDynamicBodies;

        public NativeArray<RigidBody> Bodies => m_Bodies.GetSubArray(0, NumBodies);
        public NativeArray<RigidBody> StaticBodies => m_Bodies.GetSubArray(NumDynamicBodies, NumStaticBodies);
        public NativeArray<RigidBody> DynamicBodies => m_Bodies.GetSubArray(0, NumDynamicBodies);

        // Contacts are always created between rigid bodies if they are closer than this distance threshold.
        public sfloat CollisionTolerance => sfloat.FromRaw(0x3a83126f); // todo - make this configurable?

        // Construct a collision world with the given number of uninitialized rigid bodies
        public CollisionWorld(int numStaticBodies, int numDynamicBodies)
        {
            m_Bodies = new NativeArray<RigidBody>(numStaticBodies + numDynamicBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            Broadphase = new Broadphase(numStaticBodies, numDynamicBodies);
            EntityBodyIndexMap = new NativeHashMap<Entity, int>(m_Bodies.Length, Allocator.Persistent);
        }

        internal CollisionWorld(NativeArray<RigidBody> bodies, Broadphase broadphase)
        {
            m_Bodies = bodies;
            Broadphase = broadphase;
            EntityBodyIndexMap = new NativeHashMap<Entity, int>(m_Bodies.Length, Allocator.Persistent);
        }

        public void Reset(int numStaticBodies, int numDynamicBodies)
        {
            SetCapacity(numStaticBodies + numDynamicBodies);
            Broadphase.Reset(numStaticBodies, numDynamicBodies);
            EntityBodyIndexMap.Clear();
        }

        private void SetCapacity(int numBodies)
        {
            // Increase body storage if necessary
            if (m_Bodies.Length < numBodies)
            {
                m_Bodies.Dispose();
                m_Bodies = new NativeArray<RigidBody>(numBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                EntityBodyIndexMap.Capacity = m_Bodies.Length;
            }
        }

        // Free internal memory
        public void Dispose()
        {
            if (m_Bodies.IsCreated)
            {
                m_Bodies.Dispose();
            }
            Broadphase.Dispose();
            if (EntityBodyIndexMap.IsCreated)
            {
                EntityBodyIndexMap.Dispose();
            }
        }

        // Clone the world (except the colliders)
        public CollisionWorld Clone()
        {
            var clone = new CollisionWorld
            {
                m_Bodies = new NativeArray<RigidBody>(m_Bodies, Allocator.Persistent),
                Broadphase = (Broadphase)Broadphase.Clone(),
                EntityBodyIndexMap = new NativeHashMap<Entity, int>(m_Bodies.Length, Allocator.Persistent),
            };
            clone.UpdateBodyIndexMap();
            return clone;
        }

        public void UpdateBodyIndexMap()
        {
            EntityBodyIndexMap.Clear();
            for (int i = 0; i < m_Bodies.Length; i++)
            {
                EntityBodyIndexMap[m_Bodies[i].Entity] = i;
            }
        }

        public int GetRigidBodyIndex(Entity entity)
        {
            return EntityBodyIndexMap.TryGetValue(entity, out var index) ? index : -1;
        }

        // Build the broadphase based on the given world.
        public void BuildBroadphase(ref PhysicsWorld world, sfloat timeStep, float3 gravity, bool buildStaticTree = true)
        {
            Broadphase.Build(world.StaticBodies, world.DynamicBodies, world.MotionVelocities,
                world.CollisionWorld.CollisionTolerance, timeStep, gravity, buildStaticTree);
        }

        [Obsolete("ScheduleBuildBroadphaseJobs() has been deprecated. Please use the new method taking a bool as the last parameter. (RemovedAfter 2021-02-15)", true)]
        public JobHandle ScheduleBuildBroadphaseJobs(ref PhysicsWorld world, sfloat timeStep, float3 gravity, NativeArray<int> buildStaticTree, JobHandle inputDeps, int threadCountHint = 0)
        {
            return ScheduleBuildBroadphaseJobs(ref world, timeStep, gravity, buildStaticTree, inputDeps, threadCountHint > 0);
        }

        // Schedule a set of jobs to build the broadphase based on the given world.
        public JobHandle ScheduleBuildBroadphaseJobs(ref PhysicsWorld world, sfloat timeStep, float3 gravity, NativeArray<int> buildStaticTree, JobHandle inputDeps, bool multiThreaded = true)
        {
            return Broadphase.ScheduleBuildJobs(ref world, timeStep, gravity, buildStaticTree, inputDeps, multiThreaded);
        }

        // Write all overlapping body pairs to the given streams,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public void FindOverlaps(ref NativeStream.Writer dynamicVsDynamicPairsWriter, ref NativeStream.Writer staticVsDynamicPairsWriter)
        {
            Broadphase.FindOverlaps(ref dynamicVsDynamicPairsWriter, ref staticVsDynamicPairsWriter);
        }

        [Obsolete("ScheduleFindOverlapsJobs() has been deprecated. Please use the new method taking a bool as the last parameter. (RemovedAfter 2021-02-15)", true)]
        public SimulationJobHandles ScheduleFindOverlapsJobs(out NativeStream dynamicVsDynamicPairsStream, out NativeStream staticVsDynamicPairsStream,
            JobHandle inputDeps, int threadCountHint = 0)
        {
            return ScheduleFindOverlapsJobs(out dynamicVsDynamicPairsStream, out staticVsDynamicPairsStream, inputDeps, threadCountHint > 0);
        }

        // Schedule a set of jobs which will write all overlapping body pairs to the given steam,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public SimulationJobHandles ScheduleFindOverlapsJobs(out NativeStream dynamicVsDynamicPairsStream, out NativeStream staticVsDynamicPairsStream,
            JobHandle inputDeps, bool multiThreaded = true)
        {
            return Broadphase.ScheduleFindOverlapsJobs(out dynamicVsDynamicPairsStream, out staticVsDynamicPairsStream, inputDeps, multiThreaded);
        }

        // Synchronize the collision world with the dynamics world.
        public void UpdateDynamicTree(ref PhysicsWorld world, sfloat timeStep, float3 gravity)
        {
            // Synchronize transforms
            for (int i = 0; i < world.DynamicsWorld.NumMotions; i++)
            {
                UpdateRigidBodyTransformsJob.ExecuteImpl(i, world.MotionDatas, m_Bodies);
            }

            // Update broadphase
            sfloat aabbMargin = world.CollisionWorld.CollisionTolerance * (sfloat)0.5f;
            Broadphase.BuildDynamicTree(world.DynamicBodies, world.MotionVelocities, gravity, timeStep, aabbMargin);
        }

        [Obsolete("ScheduleUpdateDynamicTree() has been deprecated. Please use the new method taking a bool as the last parameter. (RemovedAfter 2021-02-15)", true)]
        public JobHandle ScheduleUpdateDynamicTree(ref PhysicsWorld world, sfloat timeStep, float3 gravity, JobHandle inputDeps, int threadCountHint = 0)
        {
            return ScheduleUpdateDynamicTree(ref world, timeStep, gravity, inputDeps, threadCountHint > 0);
        }

        // Schedule a set of jobs to synchronize the collision world with the dynamics world.
        public JobHandle ScheduleUpdateDynamicTree(ref PhysicsWorld world, sfloat timeStep, float3 gravity, JobHandle inputDeps, bool multiThreaded = true)
        {
            if (!multiThreaded)
            {
                return new UpdateDynamicLayerJob
                {
                    World = world,
                    TimeStep = timeStep,
                    Gravity = gravity
                }.Schedule(inputDeps);
            }
            else
            {
                // Synchronize transforms
                JobHandle handle = new UpdateRigidBodyTransformsJob
                {
                    MotionDatas = world.MotionDatas,
                    RigidBodies = m_Bodies
                }.Schedule(world.MotionDatas.Length, 32, inputDeps);

                // Update broadphase
                // Thread count is +1 for main thread
                return Broadphase.ScheduleDynamicTreeBuildJobs(ref world, timeStep, gravity, JobsUtility.JobWorkerCount + 1, handle);
            }
        }

        #region Jobs

        [BurstCompile]
        private struct UpdateRigidBodyTransformsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<MotionData> MotionDatas;
            public NativeArray<RigidBody> RigidBodies;

            public void Execute(int i)
            {
                ExecuteImpl(i, MotionDatas, RigidBodies);
            }

            internal static void ExecuteImpl(int i, NativeArray<MotionData> motionDatas, NativeArray<RigidBody> rigidBodies)
            {
                RigidBody rb = rigidBodies[i];
                rb.WorldFromBody = math.mul(motionDatas[i].WorldFromMotion, math.inverse(motionDatas[i].BodyFromMotion));
                rigidBodies[i] = rb;
            }
        }

        [BurstCompile]
        private struct UpdateDynamicLayerJob : IJob
        {
            public PhysicsWorld World;
            public sfloat TimeStep;
            public float3 Gravity;

            public void Execute()
            {
                World.CollisionWorld.UpdateDynamicTree(ref World, TimeStep, Gravity);
            }
        }

        #endregion

        #region ICollidable implementation

        public Aabb CalculateAabb()
        {
            return Broadphase.Domain;
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            return TransformAabb(transform, Broadphase.Domain);
        }

        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            return Broadphase.CastRay(input, m_Bodies, ref collector);
        }

        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            return Broadphase.CastCollider(input, m_Bodies, ref collector);
        }

        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return Broadphase.CalculateDistance(input, m_Bodies, ref collector);
        }

        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            return Broadphase.CalculateDistance(input, m_Bodies, ref collector);
        }

        #region GO API Queries

        // Interfaces that represent queries that exist in the GameObjects world.

        public bool CheckSphere(float3 position, sfloat radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckSphere(ref this, position, radius, filter, queryInteraction);
        public bool OverlapSphere(float3 position, sfloat radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapSphere(ref this, position, radius, ref outHits, filter, queryInteraction);
        public bool OverlapSphereCustom<T>(float3 position, sfloat radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapSphereCustom(ref this, position, radius, ref collector, filter, queryInteraction);

        public bool CheckCapsule(float3 point1, float3 point2, sfloat radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckCapsule(ref this, point1, point2, radius, filter, queryInteraction);
        public bool OverlapCapsule(float3 point1, float3 point2, sfloat radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapCapsule(ref this, point1, point2, radius, ref outHits, filter, queryInteraction);
        public bool OverlapCapsuleCustom<T>(float3 point1, float3 point2, sfloat radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapCapsuleCustom(ref this, point1, point2, radius, ref collector, filter, queryInteraction);

        public bool CheckBox(float3 center, quaternion orientation, float3 halfExtents, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckBox(ref this, center, orientation, halfExtents, filter, queryInteraction);
        public bool OverlapBox(float3 center, quaternion orientation, float3 halfExtents, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapBox(ref this, center, orientation, halfExtents, ref outHits, filter, queryInteraction);
        public bool OverlapBoxCustom<T>(float3 center, quaternion orientation, float3 halfExtents, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapBoxCustom(ref this, center, orientation, halfExtents, ref collector, filter, queryInteraction);

        public bool SphereCast(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(ref this, origin, radius, direction, maxDistance, filter, queryInteraction);
        public bool SphereCast(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(ref this, origin, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool SphereCastAll(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCastAll(ref this, origin, radius, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool SphereCastCustom<T>(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.SphereCastCustom(ref this, origin, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        public bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(ref this, center, orientation, halfExtents, direction, maxDistance, filter, queryInteraction);
        public bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(ref this, center, orientation, halfExtents, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool BoxCastAll(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCastAll(ref this, center, orientation, halfExtents, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool BoxCastCustom<T>(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.BoxCastCustom(ref this, center, orientation, halfExtents, direction, maxDistance, ref collector, filter, queryInteraction);

        public bool CapsuleCast(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(ref this, point1, point2, radius, direction, maxDistance, filter, queryInteraction);
        public bool CapsuleCast(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(ref this, point1, point2, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool CapsuleCastAll(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCastAll(ref this, point1, point2, radius, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool CapsuleCastCustom<T>(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.CapsuleCastCustom(ref this, point1, point2, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        #endregion

        #endregion

        // Test input against the broadphase tree, filling allHits with the body indices of every overlap.
        // Returns true if there was at least overlap.
        public bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits)
        {
            int hitsBefore = allHits.Length;
            Broadphase.OverlapAabb(input, m_Bodies, ref allHits);
            return allHits.Length > hitsBefore;
        }
    }
}
