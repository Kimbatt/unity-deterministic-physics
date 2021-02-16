using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityS.Mathematics;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // A collection of rigid bodies wrapped by a bounding volume hierarchy.
    // This allows to do collision queries such as raycasting, overlap testing, etc.
    [NoAlias]
    public struct CollisionWorld : ICollidable, IDisposable
    {
        [NoAlias] private NativeArray<RigidBody> m_Bodies;    // storage for all the rigid bodies
        [NoAlias] internal Broadphase Broadphase;             // bounding volume hierarchies around subsets of the rigid bodies

        public int NumBodies => Broadphase.NumStaticBodies + Broadphase.NumDynamicBodies;
        public int NumStaticBodies => Broadphase.NumStaticBodies;
        public int NumDynamicBodies => Broadphase.NumDynamicBodies;

        public NativeArray<RigidBody> Bodies => m_Bodies.GetSubArray(0, NumBodies);
        public NativeArray<RigidBody> StaticBodies => m_Bodies.GetSubArray(NumDynamicBodies, NumStaticBodies);
        public NativeArray<RigidBody> DynamicBodies => m_Bodies.GetSubArray(0, NumDynamicBodies);

        // Contacts are always created between rigid bodies if they are closer than this distance threshold.
        public sfloat CollisionTolerance => sfloat.FromRaw(/*0x3dcccccd*/ 0x3a83126f); // todo - make this configurable?

        // Construct a collision world with the given number of uninitialized rigid bodies
        public CollisionWorld(int numStaticBodies, int numDynamicBodies)
        {
            m_Bodies = new NativeArray<RigidBody>(numStaticBodies + numDynamicBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            Broadphase = new Broadphase(numStaticBodies, numDynamicBodies);
        }

        internal CollisionWorld(NativeArray<RigidBody> bodies, Broadphase broadphase)
        {
            m_Bodies = bodies;
            Broadphase = broadphase;
        }

        public void Reset(int numStaticBodies, int numDynamicBodies)
        {
            SetCapacity(numStaticBodies + numDynamicBodies);
            Broadphase.Reset(numStaticBodies, numDynamicBodies);
        }

        private void SetCapacity(int numBodies)
        {
            // Increase body storage if necessary
            if (m_Bodies.Length < numBodies)
            {
                m_Bodies.Dispose();
                m_Bodies = new NativeArray<RigidBody>(numBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
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
        }

        // Clone the world (except the colliders)
        public CollisionWorld Clone()
        {
            return new CollisionWorld
            {
                m_Bodies = new NativeArray<RigidBody>(m_Bodies, Allocator.Persistent),
                Broadphase = (Broadphase)Broadphase.Clone()
            };
        }

        // Build the broadphase based on the given world.
        public void BuildBroadphase(ref PhysicsWorld world, sfloat timeStep, float3 gravity, bool buildStaticTree = true)
        {
            Broadphase.Build(world.StaticBodies, world.DynamicBodies, world.MotionVelocities,
                world.CollisionWorld.CollisionTolerance, timeStep, gravity, buildStaticTree);
        }

        // Schedule a set of jobs to build the broadphase based on the given world.
        public JobHandle ScheduleBuildBroadphaseJobs(ref PhysicsWorld world, sfloat timeStep, float3 gravity, NativeArray<int> buildStaticTree, JobHandle inputDeps, int threadCountHint = 0)
        {
            return Broadphase.ScheduleBuildJobs(ref world, timeStep, gravity, buildStaticTree, inputDeps, threadCountHint);
        }

        // Write all overlapping body pairs to the given streams,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public void FindOverlaps(ref NativeStream.Writer dynamicVsDynamicPairsWriter, ref NativeStream.Writer staticVsDynamicPairsWriter)
        {
            Broadphase.FindOverlaps(ref dynamicVsDynamicPairsWriter, ref staticVsDynamicPairsWriter);
        }

        // Schedule a set of jobs which will write all overlapping body pairs to the given steam,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public SimulationJobHandles ScheduleFindOverlapsJobs(out NativeStream dynamicVsDynamicPairsStream, out NativeStream staticVsDynamicPairsStream,
            JobHandle inputDeps, int threadCountHint = 0)
        {
            return Broadphase.ScheduleFindOverlapsJobs(out dynamicVsDynamicPairsStream, out staticVsDynamicPairsStream, inputDeps, threadCountHint);
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

        // Schedule a set of jobs to synchronize the collision world with the dynamics world.
        public JobHandle ScheduleUpdateDynamicTree(ref PhysicsWorld world, sfloat timeStep, float3 gravity, JobHandle inputDeps, int threadCountHint = 0)
        {
            if (threadCountHint <= 0)
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
                return Broadphase.ScheduleDynamicTreeBuildJobs(ref world, timeStep, gravity, threadCountHint, handle);
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
