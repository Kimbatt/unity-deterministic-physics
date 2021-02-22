using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using UnityS.Mathematics;
using UnityEngine.Assertions;
using static UnityS.Physics.BoundingVolumeHierarchy;

namespace UnityS.Physics
{
    // A bounding volume around a collection of rigid bodies
    [NoAlias]
    internal struct Broadphase : IDisposable
    {
        [NoAlias]
        private Tree m_StaticTree;  // The tree of static rigid bodies
        [NoAlias]
        private Tree m_DynamicTree; // The tree of dynamic rigid bodies

        public Tree StaticTree => m_StaticTree;
        public Tree DynamicTree => m_DynamicTree;
        public Aabb Domain =>
            Aabb.Union(m_StaticTree.BoundingVolumeHierarchy.Domain, m_DynamicTree.BoundingVolumeHierarchy.Domain);

        public int NumStaticBodies => m_StaticTree.NumBodies;

        public int NumDynamicBodies => m_DynamicTree.NumBodies;

        public Broadphase(int numStaticBodies, int numDynamicBodies)
        {
            m_StaticTree = new Tree(numStaticBodies);
            m_DynamicTree = new Tree(numDynamicBodies);
        }

        internal Broadphase(Tree staticTree, Tree dynamicTree)
        {
            m_StaticTree = staticTree;
            m_DynamicTree = dynamicTree;
        }

        public void Reset(int numStaticBodies, int numDynamicBodies)
        {
            m_StaticTree.Reset(numStaticBodies);
            m_DynamicTree.Reset(numDynamicBodies);
        }

        public void Dispose()
        {
            m_StaticTree.Dispose();
            m_DynamicTree.Dispose();
        }

        public Broadphase Clone()
        {
            return new Broadphase
            {
                m_StaticTree = (Tree)m_StaticTree.Clone(),
                m_DynamicTree = (Tree)m_DynamicTree.Clone()
            };
        }

        #region Build

        /// <summary>
        /// Build the broadphase based on the given world.
        /// </summary>
        public void Build(NativeArray<RigidBody> staticBodies, NativeArray<RigidBody> dynamicBodies,
            NativeArray<MotionVelocity> motionVelocities, sfloat collisionTolerance, sfloat timeStep, float3 gravity, bool buildStaticTree = true)
        {
            sfloat aabbMargin = collisionTolerance * (sfloat)0.5f; // each body contributes half

            if (buildStaticTree)
            {
                m_StaticTree.Reset(staticBodies.Length);
                BuildStaticTree(staticBodies, aabbMargin);
            }

            m_DynamicTree.Reset(dynamicBodies.Length);
            BuildDynamicTree(dynamicBodies, motionVelocities, gravity, timeStep, aabbMargin);
        }

        /// <summary>
        /// Build the static tree of the broadphase based on the given array of rigid bodies.
        /// </summary>
        public void BuildStaticTree(NativeArray<RigidBody> staticBodies, sfloat aabbMargin)
        {
            Assert.AreEqual(staticBodies.Length, m_StaticTree.NumBodies);

            if (staticBodies.Length == 0)
            {
                return;
            }

            // Read bodies
            var aabbs = new NativeArray<Aabb>(staticBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(staticBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < staticBodies.Length; i++)
            {
                PrepareStaticBodyDataJob.ExecuteImpl(i, aabbMargin, staticBodies, aabbs, points, m_StaticTree.BodyFilters, m_StaticTree.RespondsToCollision);
            }

            // Build tree
            m_StaticTree.BoundingVolumeHierarchy.Build(points, aabbs, out int nodeCount);

            // Build node filters
            m_StaticTree.BoundingVolumeHierarchy.BuildCombinedCollisionFilter(m_StaticTree.BodyFilters, 1, nodeCount - 1);
        }

        /// <summary>
        /// Build the dynamic tree of the broadphase based on the given array of rigid bodies and motions.
        /// </summary>
        public void BuildDynamicTree(NativeArray<RigidBody> dynamicBodies,
            NativeArray<MotionVelocity> motionVelocities, float3 gravity, sfloat timeStep, sfloat aabbMargin)
        {
            Assert.AreEqual(dynamicBodies.Length, m_DynamicTree.NumBodies);

            if (dynamicBodies.Length == 0)
            {
                return;
            }

            // Read bodies
            var aabbs = new NativeArray<Aabb>(dynamicBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(dynamicBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < dynamicBodies.Length; i++)
            {
                PrepareDynamicBodyDataJob.ExecuteImpl(i, aabbMargin, gravity, timeStep, dynamicBodies, motionVelocities, aabbs, points,
                    m_DynamicTree.BodyFilters, m_DynamicTree.RespondsToCollision);
            }

            // Build tree
            m_DynamicTree.BoundingVolumeHierarchy.Build(points, aabbs, out int nodeCount);

            // Build node filters
            m_DynamicTree.BoundingVolumeHierarchy.BuildCombinedCollisionFilter(m_DynamicTree.BodyFilters, 1, nodeCount - 1);
        }

        /// <summary>
        /// Schedule a set of jobs to build the broadphase based on the given world.
        /// </summary>
        public JobHandle ScheduleBuildJobs(ref PhysicsWorld world, sfloat timeStep, float3 gravity, NativeArray<int> buildStaticTree, JobHandle inputDeps, bool multiThreaded = true)
        {
            if (!multiThreaded)
            {
                return new BuildBroadphaseJob
                {
                    StaticBodies = world.StaticBodies,
                    DynamicBodies = world.DynamicBodies,
                    MotionVelocities = world.MotionVelocities,
                    CollisionTolerance = world.CollisionWorld.CollisionTolerance,
                    TimeStep = timeStep,
                    Gravity = gravity,
                    BuildStaticTree = buildStaticTree,
                    Broadphase = this
                }.Schedule(inputDeps);
            }
            else
            {
                // +1 for main thread
                int threadCount = JobsUtility.JobWorkerCount + 1;
                return JobHandle.CombineDependencies(
                    ScheduleStaticTreeBuildJobs(ref world, threadCount, buildStaticTree, inputDeps),
                    ScheduleDynamicTreeBuildJobs(ref world, timeStep, gravity, threadCount, inputDeps));
            }
        }

        /// <summary>
        /// Schedule a set of jobs to build the static tree of the broadphase based on the given world.
        /// </summary>
        public JobHandle ScheduleStaticTreeBuildJobs(
            ref PhysicsWorld world, int numThreadsHint, NativeArray<int> shouldDoWork, JobHandle inputDeps)
        {
            Assert.AreEqual(world.NumStaticBodies, m_StaticTree.NumBodies);
            if (world.NumStaticBodies == 0)
            {
                return inputDeps;
            }

            var aabbs = new NativeArray<Aabb>(world.NumStaticBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(world.NumStaticBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var numStaticBodiesArray = new NativeArray<int>(1, Allocator.TempJob);
            JobHandle handle = new PrepareNumStaticBodiesJob
            {
                NumStaticBodies = world.NumStaticBodies,
                BuildStaticTree = shouldDoWork,
                NumStaticBodiesArray = numStaticBodiesArray
            }.Schedule(inputDeps);

            handle = new PrepareStaticBodyDataJob
            {
                RigidBodies = world.StaticBodies,
                Aabbs = aabbs,
                Points = points,
                FiltersOut = m_StaticTree.BodyFilters,
                RespondsToCollisionOut = m_StaticTree.RespondsToCollision,
                AabbMargin = world.CollisionWorld.CollisionTolerance * (sfloat)0.5f, // each body contributes half
            }.ScheduleUnsafeIndex0(numStaticBodiesArray, 32, handle);

            var buildHandle = m_StaticTree.BoundingVolumeHierarchy.ScheduleBuildJobs(
                points, aabbs, m_StaticTree.BodyFilters, shouldDoWork, numThreadsHint, handle,
                m_StaticTree.Nodes.Length, m_StaticTree.Ranges, m_StaticTree.BranchCount);

            return JobHandle.CombineDependencies(buildHandle, numStaticBodiesArray.Dispose(handle));
        }

        /// <summary>
        /// Schedule a set of jobs to build the dynamic tree of the broadphase based on the given world.
        /// </summary>
        public JobHandle ScheduleDynamicTreeBuildJobs(
            ref PhysicsWorld world, sfloat timeStep, float3 gravity, int numThreadsHint, JobHandle inputDeps)
        {
            Assert.AreEqual(world.NumDynamicBodies, m_DynamicTree.NumBodies);
            if (world.NumDynamicBodies == 0)
            {
                return inputDeps;
            }

            var aabbs = new NativeArray<Aabb>(world.NumDynamicBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(world.NumDynamicBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            JobHandle handle = new PrepareDynamicBodyDataJob
            {
                RigidBodies = world.DynamicBodies,
                MotionVelocities = world.MotionVelocities,
                Aabbs = aabbs,
                Points = points,
                FiltersOut = m_DynamicTree.BodyFilters,
                RespondsToCollisionOut = m_DynamicTree.RespondsToCollision,
                AabbMargin = world.CollisionWorld.CollisionTolerance * (sfloat)0.5f, // each body contributes half
                TimeStep = timeStep,
                Gravity = gravity
            }.Schedule(world.NumDynamicBodies, 32, inputDeps);

            var shouldDoWork = new NativeArray<int>(1, Allocator.TempJob);
            shouldDoWork[0] = 1;

            handle = m_DynamicTree.BoundingVolumeHierarchy.ScheduleBuildJobs(
                points, aabbs, m_DynamicTree.BodyFilters, shouldDoWork, numThreadsHint, handle,
                m_DynamicTree.Nodes.Length, m_DynamicTree.Ranges, m_DynamicTree.BranchCount);

            return shouldDoWork.Dispose(handle);
        }

        #endregion

        #region Find overlaps

        // Write all overlapping body pairs to the given streams,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public void FindOverlaps(ref NativeStream.Writer dynamicVsDynamicPairsWriter, ref NativeStream.Writer staticVsDynamicPairsWriter)
        {
            // Dynamic-dynamic
            {
                dynamicVsDynamicPairsWriter.BeginForEachIndex(0);
                DynamicVsDynamicFindOverlappingPairsJob.ExecuteImpl(
                    new int2(1, 1), m_DynamicTree, ref dynamicVsDynamicPairsWriter);
                dynamicVsDynamicPairsWriter.EndForEachIndex();
            }

            // Static-dynamic
            {
                staticVsDynamicPairsWriter.BeginForEachIndex(0);
                StaticVsDynamicFindOverlappingPairsJob.ExecuteImpl(
                    new int2(1, 1), m_StaticTree, m_DynamicTree, ref staticVsDynamicPairsWriter);
                staticVsDynamicPairsWriter.EndForEachIndex();
            }
        }

        // Schedule a set of jobs which will write all overlapping body pairs to the given steam,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public SimulationJobHandles ScheduleFindOverlapsJobs(out NativeStream dynamicVsDynamicPairsStream, out NativeStream staticVsDynamicPairsStream,
            JobHandle inputDeps, bool multiThreaded = true)
        {
            SimulationJobHandles returnHandles = default;

            if (!multiThreaded)
            {
                dynamicVsDynamicPairsStream = new NativeStream(1, Allocator.TempJob);
                staticVsDynamicPairsStream = new NativeStream(1, Allocator.TempJob);
                returnHandles.FinalExecutionHandle = new FindOverlapsJob
                {
                    Broadphase = this,
                    DynamicVsDynamicPairsWriter = dynamicVsDynamicPairsStream.AsWriter(),
                    StaticVsDynamicPairsWriter = staticVsDynamicPairsStream.AsWriter()
                }.Schedule(inputDeps);

                return returnHandles;
            }

            var dynamicVsDynamicNodePairIndices = new NativeList<int2>(Allocator.TempJob);
            var staticVsDynamicNodePairIndices = new NativeList<int2>(Allocator.TempJob);

            JobHandle allocateDeps = new AllocateDynamicVsStaticNodePairs
            {
                dynamicVsDynamicNodePairIndices = dynamicVsDynamicNodePairIndices,
                staticVsDynamicNodePairIndices = staticVsDynamicNodePairIndices,
                dynamicBranchCount = m_DynamicTree.BranchCount,
                staticBranchCount = m_StaticTree.BranchCount
            }.Schedule(inputDeps);

            // Build pairs of branch node indices
            JobHandle dynamicVsDynamicPairs = new DynamicVsDynamicBuildBranchNodePairsJob
            {
                Ranges = m_DynamicTree.Ranges,
                NumBranches = m_DynamicTree.BranchCount,
                NodePairIndices = dynamicVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(allocateDeps);

            JobHandle staticVsDynamicPairs = new StaticVsDynamicBuildBranchNodePairsJob
            {
                DynamicRanges = m_DynamicTree.Ranges,
                StaticRanges = m_StaticTree.Ranges,
                NumStaticBranches = m_StaticTree.BranchCount,
                NumDynamicBranches = m_DynamicTree.BranchCount,
                NodePairIndices = staticVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(allocateDeps);

            //@TODO: We only need a dependency on allocateDeps, but the safety system doesn't understand that we can not change length list in DynamicVsDynamicBuildBranchNodePairsJob & StaticVsDynamicBuildBranchNodePairsJob
            //       if this is a performance issue we can use [NativeDisableContainerSafetyRestriction] on DynamicVsDynamicBuildBranchNodePairsJob & StaticVsDynamicBuildBranchNodePairsJob
            JobHandle dynamicConstruct = NativeStream.ScheduleConstruct(out dynamicVsDynamicPairsStream, dynamicVsDynamicNodePairIndices, dynamicVsDynamicPairs, Allocator.TempJob);
            JobHandle staticConstruct = NativeStream.ScheduleConstruct(out staticVsDynamicPairsStream, staticVsDynamicNodePairIndices, staticVsDynamicPairs, Allocator.TempJob);

            // Write all overlaps to the stream (also deallocates nodePairIndices)
            JobHandle dynamicVsDynamicHandle = new DynamicVsDynamicFindOverlappingPairsJob
            {
                DynamicTree = m_DynamicTree,
                PairWriter = dynamicVsDynamicPairsStream.AsWriter(),
                NodePairIndices = dynamicVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(dynamicVsDynamicNodePairIndices, 1, JobHandle.CombineDependencies(dynamicVsDynamicPairs, dynamicConstruct));

            // Write all overlaps to the stream (also deallocates nodePairIndices)
            JobHandle staticVsDynamicHandle = new StaticVsDynamicFindOverlappingPairsJob
            {
                StaticTree = m_StaticTree,
                DynamicTree = m_DynamicTree,
                PairWriter = staticVsDynamicPairsStream.AsWriter(),
                NodePairIndices = staticVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(staticVsDynamicNodePairIndices, 1, JobHandle.CombineDependencies(staticVsDynamicPairs, staticConstruct));

            // Dispose node pair lists
            var disposeOverlapPairs0 = dynamicVsDynamicNodePairIndices.Dispose(dynamicVsDynamicHandle);
            var disposeOverlapPairs1 = staticVsDynamicNodePairIndices.Dispose(staticVsDynamicHandle);
            returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(disposeOverlapPairs0, disposeOverlapPairs1);
            returnHandles.FinalExecutionHandle = JobHandle.CombineDependencies(dynamicVsDynamicHandle, staticVsDynamicHandle);

            return returnHandles;
        }

        #endregion

        #region Tree

        // A tree of rigid bodies
        [NoAlias]
        public struct Tree : IDisposable
        {
            [NoAlias] public NativeArray<Node> Nodes; // The nodes of the bounding volume
            [NoAlias] public NativeArray<CollisionFilter> NodeFilters; // The collision filter for each node (a union of all its children)
            [NoAlias] public NativeArray<CollisionFilter> BodyFilters; // A copy of the collision filter of each body
            [NoAlias] public NativeArray<bool> RespondsToCollision; // A copy of the RespondsToCollision flag of each body
            [NoAlias] internal NativeArray<Builder.Range> Ranges; // Used during building
            [NoAlias] internal NativeArray<int> BranchCount; // Used during building
            internal Allocator Allocator;

            public BoundingVolumeHierarchy BoundingVolumeHierarchy => new BoundingVolumeHierarchy(Nodes, NodeFilters);

            public int NumBodies => BodyFilters.Length;

            public Tree(int numBodies, Allocator allocator = Allocator.Persistent)
            {
                this = default;
                Allocator = allocator;
                SetCapacity(numBodies);
                Ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(
                    BoundingVolumeHierarchy.Constants.MaxNumTreeBranches, allocator, NativeArrayOptions.UninitializedMemory);
                BranchCount = new NativeArray<int>(1, allocator, NativeArrayOptions.ClearMemory);
            }

            public void Reset(int numBodies)
            {
                if (numBodies != BodyFilters.Length)
                {
                    SetCapacity(numBodies);
                }
            }

            private void SetCapacity(int numBodies)
            {
                int numNodes = numBodies + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches;

                if (Nodes.IsCreated)
                {
                    Nodes.Dispose();
                }
                Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(numNodes, Allocator, NativeArrayOptions.UninitializedMemory)
                {
                    // Always initialize first 2 nodes as empty, to gracefully return from queries on an empty tree
                    [0] = BoundingVolumeHierarchy.Node.Empty,
                    [1] = BoundingVolumeHierarchy.Node.Empty
                };

                if (NodeFilters.IsCreated)
                {
                    NodeFilters.Dispose();
                }
                NodeFilters = new NativeArray<CollisionFilter>(numNodes, Allocator, NativeArrayOptions.UninitializedMemory)
                {
                    // All queries should descend past these special root nodes
                    [0] = CollisionFilter.Default,
                    [1] = CollisionFilter.Default
                };

                if (BodyFilters.IsCreated)
                {
                    BodyFilters.Dispose();
                }
                BodyFilters = new NativeArray<CollisionFilter>(numBodies, Allocator, NativeArrayOptions.UninitializedMemory);

                if (RespondsToCollision.IsCreated)
                {
                    RespondsToCollision.Dispose();
                }
                RespondsToCollision = new NativeArray<bool>(numBodies, Allocator, NativeArrayOptions.UninitializedMemory);
            }

            public Tree Clone()
            {
                return new Tree
                {
                    Allocator = Allocator,
                    Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(Nodes, Allocator),
                    NodeFilters = new NativeArray<CollisionFilter>(NodeFilters, Allocator),
                    BodyFilters = new NativeArray<CollisionFilter>(BodyFilters, Allocator),
                    RespondsToCollision = new NativeArray<bool>(RespondsToCollision, Allocator),
                    Ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(Ranges, Allocator),
                    BranchCount = new NativeArray<int>(BranchCount, Allocator)
                };
            }

            public void Dispose()
            {
                if (Nodes.IsCreated)
                {
                    Nodes.Dispose();
                }

                if (NodeFilters.IsCreated)
                {
                    NodeFilters.Dispose();
                }

                if (BodyFilters.IsCreated)
                {
                    BodyFilters.Dispose();
                }

                if (RespondsToCollision.IsCreated)
                {
                    RespondsToCollision.Dispose();
                }

                if (Ranges.IsCreated)
                {
                    Ranges.Dispose();
                }

                if (BranchCount.IsCreated)
                {
                    BranchCount.Dispose();
                }
            }
        }

        #endregion

        #region Queries

        internal struct RigidBodyOverlapsCollector : IOverlapCollector
        {
            public NativeList<int> RigidBodyIndices;

            public unsafe void AddRigidBodyIndices(int* indices, int count)
            {
                RigidBodyIndices.AddRange(indices, count);
            }

            public unsafe void AddColliderKeys(ColliderKey* keys, int count) => SafetyChecks.ThrowNotSupportedException();

            public void PushCompositeCollider(ColliderKeyPath compositeKey) => SafetyChecks.ThrowNotSupportedException();

            public void PopCompositeCollider(uint numCompositeKeyBits) => SafetyChecks.ThrowNotSupportedException();
        }

        // Test broadphase nodes against the aabb in input. For any overlapping
        // tree leaves, put the body indices into the output leafIndices.
        public void OverlapAabb(OverlapAabbInput input, NativeArray<RigidBody> rigidBodies, ref NativeList<int> rigidBodyIndices)
        {
            if (input.Filter.IsEmpty)
                return;
            var leafProcessor = new BvhLeafProcessor(rigidBodies);
            var leafCollector = new RigidBodyOverlapsCollector { RigidBodyIndices = rigidBodyIndices };

            leafProcessor.BaseRigidBodyIndex = m_DynamicTree.NumBodies;
            m_StaticTree.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref leafCollector);

            leafProcessor.BaseRigidBodyIndex = 0;
            m_DynamicTree.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref leafCollector);
        }

        public bool CastRay<T>(RaycastInput input, NativeArray<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<RaycastHit>
        {
            if (input.Filter.IsEmpty)
                return false;
            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            leafProcessor.BaseRigidBodyIndex = m_DynamicTree.NumBodies;
            bool hasHit = m_StaticTree.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);

            leafProcessor.BaseRigidBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);

            return hasHit;
        }

        public unsafe bool CastCollider<T>(ColliderCastInput input, NativeArray<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            Assert.IsTrue(input.Collider != null);
            if (input.Collider->Filter.IsEmpty)
                return false;
            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            leafProcessor.BaseRigidBodyIndex = m_DynamicTree.NumBodies;
            bool hasHit = m_StaticTree.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);

            leafProcessor.BaseRigidBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);

            return hasHit;
        }

        public bool CalculateDistance<T>(PointDistanceInput input, NativeArray<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            if (input.Filter.IsEmpty)
                return false;
            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            leafProcessor.BaseRigidBodyIndex = m_DynamicTree.NumBodies;
            bool hasHit = m_StaticTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            leafProcessor.BaseRigidBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            return hasHit;
        }

        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, NativeArray<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            Assert.IsTrue(input.Collider != null);
            if (input.Collider->Filter.IsEmpty)
                return false;
            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            leafProcessor.BaseRigidBodyIndex = m_DynamicTree.NumBodies;
            bool hasHit = m_StaticTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            leafProcessor.BaseRigidBodyIndex = 0;
            hasHit |= m_DynamicTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            return hasHit;
        }

        internal struct BvhLeafProcessor :
            BoundingVolumeHierarchy.IRaycastLeafProcessor,
                BoundingVolumeHierarchy.IColliderCastLeafProcessor,
                BoundingVolumeHierarchy.IPointDistanceLeafProcessor,
                BoundingVolumeHierarchy.IColliderDistanceLeafProcessor,
                BoundingVolumeHierarchy.IAabbOverlapLeafProcessor
        {
            private readonly NativeArray<RigidBody> m_Bodies;
            public int BaseRigidBodyIndex;

            public BvhLeafProcessor(NativeArray<RigidBody> bodies)
            {
                m_Bodies = bodies;
                BaseRigidBodyIndex = 0;
            }

            public bool AabbOverlap(int rigidBodyIndex, ref NativeList<int> allHits)
            {
                allHits.Add(BaseRigidBodyIndex + rigidBodyIndex);
                return true;
            }

            public bool RayLeaf<T>(RaycastInput input, int rigidBodyIndex, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                rigidBodyIndex += BaseRigidBodyIndex;

                input.QueryContext.IsInitialized = true;
                input.QueryContext.RigidBodyIndex = rigidBodyIndex;

                RigidBody body = m_Bodies[rigidBodyIndex];

                return body.CastRay(input, ref collector);
            }

            public unsafe bool ColliderCastLeaf<T>(ColliderCastInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                rigidBodyIndex += BaseRigidBodyIndex;

                input.QueryContext.IsInitialized = true;
                input.QueryContext.RigidBodyIndex = rigidBodyIndex;

                RigidBody body = m_Bodies[rigidBodyIndex];

                return body.CastCollider(input, ref collector);
            }

            public bool DistanceLeaf<T>(PointDistanceInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                rigidBodyIndex += BaseRigidBodyIndex;

                input.QueryContext.IsInitialized = true;
                input.QueryContext.RigidBodyIndex = rigidBodyIndex;

                RigidBody body = m_Bodies[rigidBodyIndex];

                return body.CalculateDistance(input, ref collector);
            }

            public unsafe bool DistanceLeaf<T>(ColliderDistanceInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                rigidBodyIndex += BaseRigidBodyIndex;

                input.QueryContext.IsInitialized = true;
                input.QueryContext.RigidBodyIndex = rigidBodyIndex;

                RigidBody body = m_Bodies[rigidBodyIndex];

                return body.CalculateDistance(input, ref collector);
            }

            public unsafe void AabbLeaf<T>(OverlapAabbInput input, int rigidBodyIndex, ref T collector)
                where T : struct, IOverlapCollector
            {
                rigidBodyIndex += BaseRigidBodyIndex;
                RigidBody body = m_Bodies[rigidBodyIndex];
                if (body.Collider.IsCreated && CollisionFilter.IsCollisionEnabled(input.Filter, body.Collider.Value.Filter))
                {
                    collector.AddRigidBodyIndices(&rigidBodyIndex, 1);
                }
            }
        }

        #endregion

        #region Jobs

        // Builds the broadphase in a single job.
        [BurstCompile]
        struct BuildBroadphaseJob : IJob
        {
            [ReadOnly] public NativeArray<RigidBody> StaticBodies;
            [ReadOnly] public NativeArray<RigidBody> DynamicBodies;
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public sfloat CollisionTolerance;
            [ReadOnly] public sfloat TimeStep;
            [ReadOnly] public float3 Gravity;
            [ReadOnly] public NativeArray<int> BuildStaticTree;

            public Broadphase Broadphase;

            public void Execute()
            {
                Broadphase.Build(StaticBodies, DynamicBodies, MotionVelocities, CollisionTolerance, TimeStep, Gravity, BuildStaticTree[0] == 1);
            }
        }

        // Writes pairs of overlapping broadphase AABBs to a stream, in a single job.
        [BurstCompile]
        struct FindOverlapsJob : IJob
        {
            [ReadOnly] public Broadphase Broadphase;
            public NativeStream.Writer DynamicVsDynamicPairsWriter;
            public NativeStream.Writer StaticVsDynamicPairsWriter;

            public void Execute()
            {
                Broadphase.FindOverlaps(ref DynamicVsDynamicPairsWriter, ref StaticVsDynamicPairsWriter);
            }
        }

        // Allocate memory for pair indices
        [BurstCompile]
        struct AllocateDynamicVsStaticNodePairs : IJob
        {
            [ReadOnly] public NativeArray<int> dynamicBranchCount;
            [ReadOnly] public NativeArray<int> staticBranchCount;

            public NativeList<int2> dynamicVsDynamicNodePairIndices;
            public NativeList<int2> staticVsDynamicNodePairIndices;

            public void Execute()
            {
                int numDynamicVsDynamicBranchOverlapPairs = dynamicBranchCount[0] * (dynamicBranchCount[0] + 1) / 2;
                dynamicVsDynamicNodePairIndices.ResizeUninitialized(numDynamicVsDynamicBranchOverlapPairs);

                int numStaticVsDynamicBranchOverlapPairs = staticBranchCount[0] * dynamicBranchCount[0];
                staticVsDynamicNodePairIndices.ResizeUninitialized(numStaticVsDynamicBranchOverlapPairs);
            }
        }

        // Reads broadphase data from dynamic rigid bodies
        [BurstCompile]
        struct PrepareDynamicBodyDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<RigidBody> RigidBodies;
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public sfloat TimeStep;
            [ReadOnly] public float3 Gravity;
            [ReadOnly] public sfloat AabbMargin;

            public NativeArray<PointAndIndex> Points;
            public NativeArray<Aabb> Aabbs;
            public NativeArray<CollisionFilter> FiltersOut;
            public NativeArray<bool> RespondsToCollisionOut;

            public unsafe void Execute(int index)
            {
                ExecuteImpl(index, AabbMargin, Gravity, TimeStep, RigidBodies, MotionVelocities, Aabbs, Points, FiltersOut, RespondsToCollisionOut);
            }

            internal static unsafe void ExecuteImpl(int index, sfloat aabbMargin, float3 gravity, sfloat timeStep,
                NativeArray<RigidBody> rigidBodies, NativeArray<MotionVelocity> motionVelocities,
                NativeArray<Aabb> aabbs, NativeArray<PointAndIndex> points,
                NativeArray<CollisionFilter> filtersOut, NativeArray<bool> respondsToCollisionOut)
            {
                RigidBody body = rigidBodies[index];

                Aabb aabb;
                if (body.Collider.IsCreated)
                {
                    var mv = motionVelocities[index];

                    // Apply gravity only on a copy to get proper expansion for the AABB,
                    // actual applying of gravity will be done later in the physics step
                    mv.LinearVelocity += gravity * timeStep * mv.GravityFactor;

                    MotionExpansion expansion = mv.CalculateExpansion(timeStep);
                    aabb = expansion.ExpandAabb(body.Collider.Value.CalculateAabb(body.WorldFromBody));
                    aabb.Expand(aabbMargin);

                    filtersOut[index] = body.Collider.Value.Filter;
                    respondsToCollisionOut[index] = body.Collider.Value.RespondsToCollision;
                }
                else
                {
                    aabb.Min = body.WorldFromBody.pos;
                    aabb.Max = body.WorldFromBody.pos;

                    filtersOut[index] = CollisionFilter.Zero;
                    respondsToCollisionOut[index] = false;
                }

                aabbs[index] = aabb;
                points[index] = new BoundingVolumeHierarchy.PointAndIndex
                {
                    Position = aabb.Center,
                    Index = index
                };
            }
        }

        // Prepares the NumStaticBodies value for PrepareStaticBodyDataJob
        [BurstCompile]
        struct PrepareNumStaticBodiesJob : IJob
        {
            public int NumStaticBodies;
            public NativeArray<int> BuildStaticTree;
            public NativeArray<int> NumStaticBodiesArray;

            public void Execute()
            {
                if (BuildStaticTree[0] == 1)
                {
                    NumStaticBodiesArray[0] = NumStaticBodies;
                }
                else
                {
                    NumStaticBodiesArray[0] = 0;
                }
            }
        }

        // Reads broadphase data from static rigid bodies
        [BurstCompile]
        struct PrepareStaticBodyDataJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeArray<RigidBody> RigidBodies;
            [ReadOnly] public sfloat AabbMargin;

            public NativeArray<Aabb> Aabbs;
            public NativeArray<PointAndIndex> Points;
            public NativeArray<CollisionFilter> FiltersOut;
            public NativeArray<bool> RespondsToCollisionOut;

            public unsafe void Execute(int index)
            {
                ExecuteImpl(index, AabbMargin, RigidBodies, Aabbs, Points, FiltersOut, RespondsToCollisionOut);
            }

            internal static unsafe void ExecuteImpl(int index, sfloat aabbMargin,
                NativeArray<RigidBody> rigidBodies, NativeArray<Aabb> aabbs, NativeArray<PointAndIndex> points,
                NativeArray<CollisionFilter> filtersOut, NativeArray<bool> respondsToCollisionOut)
            {
                RigidBody body = rigidBodies[index];

                Aabb aabb;
                if (body.Collider.IsCreated)
                {
                    aabb = body.Collider.Value.CalculateAabb(body.WorldFromBody);
                    aabb.Expand(aabbMargin);

                    filtersOut[index] = body.Collider.Value.Filter;
                    respondsToCollisionOut[index] = body.Collider.Value.RespondsToCollision;
                }
                else
                {
                    aabb.Min = body.WorldFromBody.pos;
                    aabb.Max = body.WorldFromBody.pos;

                    filtersOut[index] = CollisionFilter.Zero;
                    respondsToCollisionOut[index] = false;
                }

                aabbs[index] = aabb;
                points[index] = new BoundingVolumeHierarchy.PointAndIndex
                {
                    Position = aabb.Center,
                    Index = index
                };
            }
        }

        // Builds a list of branch node index pairs (an input to FindOverlappingPairsJob)
        [BurstCompile]
        internal struct DynamicVsDynamicBuildBranchNodePairsJob : IJob
        {
            [ReadOnly] public NativeArray<Builder.Range> Ranges;
            [ReadOnly] public NativeArray<int> NumBranches;
            public NativeArray<int2> NodePairIndices;

            public void Execute()
            {
                int numBranches = NumBranches[0];

                int arrayIndex = 0;

                // First add all branch self overlaps.
                // Start with largest branch
                for (int i = 0; i < numBranches; i++)
                {
                    NodePairIndices[arrayIndex++] = new int2(Ranges[i].Root, Ranges[i].Root);
                }

                for (int i = 0; i < numBranches; i++)
                {
                    for (int j = i + 1; j < numBranches; j++)
                    {
                        var pair = new int2 { x = Ranges[i].Root, y = Ranges[j].Root };
                        NodePairIndices[arrayIndex++] = pair;
                    }
                }
            }
        }

        // Builds a list of branch node index pairs (an input to FindOverlappingPairsJob)
        [BurstCompile]
        struct StaticVsDynamicBuildBranchNodePairsJob : IJob
        {
            [ReadOnly] public NativeArray<Builder.Range> StaticRanges;
            [ReadOnly] public NativeArray<Builder.Range> DynamicRanges;
            [ReadOnly] public NativeArray<int> NumStaticBranches;
            [ReadOnly] public NativeArray<int> NumDynamicBranches;
            public NativeArray<int2> NodePairIndices;

            public void Execute()
            {
                int numStaticBranches = NumStaticBranches[0];
                int numDynamicBranches = NumDynamicBranches[0];

                int arrayIndex = 0;
                for (int i = 0; i < numStaticBranches; i++)
                {
                    for (int j = 0; j < numDynamicBranches; j++)
                    {
                        var pair = new int2 { x = StaticRanges[i].Root, y = DynamicRanges[j].Root };
                        NodePairIndices[arrayIndex++] = pair;
                    }
                }
            }
        }

        // An implementation of IOverlapCollector which filters and writes body pairs to a native stream
        internal unsafe struct BodyPairWriter : ITreeOverlapCollector
        {
            const int k_Capacity = 256;
            const int k_Margin = 64;
            const int k_Threshold = k_Capacity - k_Margin;

            fixed int m_PairsLeft[k_Capacity];
            fixed int m_PairsRight[k_Capacity];

            private readonly NativeStream.Writer* m_CollidingPairs;
            private readonly CollisionFilter* m_BodyFiltersLeft;
            private readonly CollisionFilter* m_BodyFiltersRight;
            private readonly bool* m_BodyRespondsToCollisionLeft;
            private readonly bool* m_BodyRespondsToCollisionRight;
            private readonly int m_BodyIndexABase;
            private readonly int m_BodyIndexBBase;
            private int m_Count;

            public BodyPairWriter(NativeStream.Writer* collidingPairs, CollisionFilter* bodyFiltersLeft, CollisionFilter* bodyFiltersRight,
                                  bool* bodyRespondsToCollisionLeft, bool* bodyRespondsToCollisionRight, int bodyIndexABase, int bodyIndexBBase)
            {
                m_CollidingPairs = collidingPairs;
                m_BodyFiltersLeft = bodyFiltersLeft;
                m_BodyFiltersRight = bodyFiltersRight;
                m_BodyRespondsToCollisionLeft = bodyRespondsToCollisionLeft;
                m_BodyRespondsToCollisionRight = bodyRespondsToCollisionRight;
                m_BodyIndexABase = bodyIndexABase;
                m_BodyIndexBBase = bodyIndexBBase;
                m_Count = 0;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AddPairs(int4 pairsLeft, int4 pairsRight, int count, bool swapped = false)
            {
                if (swapped)
                {
                    fixed (int* l = m_PairsRight)
                    {
                        *((int4*)(l + m_Count)) = pairsLeft;
                    }

                    fixed (int* r = m_PairsLeft)
                    {
                        *((int4*)(r + m_Count)) = pairsRight;
                    }
                }
                else
                {
                    fixed (int* l = m_PairsLeft)
                    {
                        *((int4*)(l + m_Count)) = pairsLeft;
                    }

                    fixed (int* r = m_PairsRight)
                    {
                        *((int4*)(r + m_Count)) = pairsRight;
                    }
                }

                m_Count += count;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AddPairs(int pairLeft, int4 pairsRight, int countR)
            {
                fixed (int* l = m_PairsLeft)
                {
                    *((int4*)(l + m_Count)) = new int4(pairLeft);
                }

                fixed (int* r = m_PairsRight)
                {
                    *((int4*)(r + m_Count)) = pairsRight;
                }

                m_Count += countR;
            }

            public void Close()
            {
                Flush();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void FlushIfNeeded()
            {
                if (m_Count >= k_Threshold)
                {
                    Flush();
                }
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            private void Flush()
            {
                if (m_Count != 0)
                {
                    fixed (int* l = m_PairsLeft)
                    {
                        fixed (int* r = m_PairsRight)
                        {
                            for (int i = 0; i < m_Count; i++)
                            {
                                int bodyALocalIndex = l[i];
                                int bodyBLocalIndex = r[i];
                                if (m_BodyRespondsToCollisionLeft[bodyALocalIndex] && m_BodyRespondsToCollisionRight[bodyBLocalIndex])
                                {
                                    if (CollisionFilter.IsCollisionEnabled(m_BodyFiltersLeft[bodyALocalIndex], m_BodyFiltersRight[bodyBLocalIndex]))
                                    {
                                        m_CollidingPairs->Write(new BodyIndexPair
                                        {
                                            BodyIndexA = bodyALocalIndex + m_BodyIndexABase,
                                            BodyIndexB = bodyBLocalIndex + m_BodyIndexBBase
                                        });
                                    }
                                }
                            }
                        }
                    }

                    m_Count = 0;
                }
            }
        }

        // Writes pairs of overlapping broadphase AABBs to a stream.
        [BurstCompile]
        internal struct DynamicVsDynamicFindOverlappingPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public Tree DynamicTree;
            [ReadOnly] public NativeArray<int2> NodePairIndices;
            public NativeStream.Writer PairWriter;

            public void Execute(int index)
            {
                PairWriter.BeginForEachIndex(index);

                int2 pair = NodePairIndices[index];
                ExecuteImpl(pair, DynamicTree, ref PairWriter);

                PairWriter.EndForEachIndex();
            }

            internal static unsafe void ExecuteImpl(int2 pair, Tree dynamicTree, ref NativeStream.Writer pairWriter)
            {
                var bodyFilters = (CollisionFilter*)dynamicTree.BodyFilters.GetUnsafeReadOnlyPtr();
                var bodyRespondsToCollision = (bool*)dynamicTree.RespondsToCollision.GetUnsafeReadOnlyPtr();
                var bufferedPairs = new BodyPairWriter((NativeStream.Writer*)UnsafeUtility.AddressOf(ref pairWriter), bodyFilters, bodyFilters, bodyRespondsToCollision, bodyRespondsToCollision, 0, 0);
                new BoundingVolumeHierarchy(dynamicTree.Nodes, dynamicTree.NodeFilters).SelfBvhOverlap(ref bufferedPairs, pair.x, pair.y);
                bufferedPairs.Close();
            }
        }

        // Writes pairs of overlapping broadphase AABBs to a stream.
        [BurstCompile]
        struct StaticVsDynamicFindOverlappingPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public Tree StaticTree;
            [ReadOnly] public Tree DynamicTree;
            [ReadOnly] public NativeArray<int2> NodePairIndices;
            public NativeStream.Writer PairWriter;

            public void Execute(int index)
            {
                PairWriter.BeginForEachIndex(index);

                int2 pair = NodePairIndices[index];
                ExecuteImpl(pair, StaticTree, DynamicTree, ref PairWriter);

                PairWriter.EndForEachIndex();
            }

            internal static unsafe void ExecuteImpl(int2 pair, Tree staticTree, Tree dynamicTree, ref NativeStream.Writer pairWriter)
            {
                var staticBvh = new BoundingVolumeHierarchy(staticTree.Nodes, staticTree.NodeFilters);
                var dynamicBvh = new BoundingVolumeHierarchy(dynamicTree.Nodes, dynamicTree.NodeFilters);

                var bodyPairWriter = new BodyPairWriter((NativeStream.Writer*)UnsafeUtility.AddressOf(ref pairWriter),
                    (CollisionFilter*)staticTree.BodyFilters.GetUnsafeReadOnlyPtr(), (CollisionFilter*)dynamicTree.BodyFilters.GetUnsafeReadOnlyPtr(),
                    (bool*)staticTree.RespondsToCollision.GetUnsafeReadOnlyPtr(), (bool*)dynamicTree.RespondsToCollision.GetUnsafeReadOnlyPtr(),
                    dynamicTree.NumBodies, 0);

                staticBvh.BvhOverlap(ref bodyPairWriter, dynamicBvh, pair.x, pair.y);

                bodyPairWriter.Close();
            }
        }

        #endregion
    }
}
