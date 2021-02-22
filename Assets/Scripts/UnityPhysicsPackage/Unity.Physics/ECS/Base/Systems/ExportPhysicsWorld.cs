using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;
using UnityS.Transforms;

namespace UnityS.Physics.Systems
{
    // A system which copies transforms and velocities from the physics world back to the original entity components.
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class ExportPhysicsWorld : SystemBase, IPhysicsSystem
    {
        private JobHandle m_InputDependency;
        private JobHandle m_OutputDependency;

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;

        internal unsafe struct SharedData : IDisposable
        {
            [NativeDisableUnsafePtrRestriction]
            public AtomicSafetyManager* SafetyManager;

            public static SharedData Create()
            {
                var sharedData = new SharedData();
                sharedData.SafetyManager = (AtomicSafetyManager*)UnsafeUtility.Malloc(UnsafeUtility.SizeOf<AtomicSafetyManager>(), 16, Allocator.Persistent);
                *sharedData.SafetyManager = AtomicSafetyManager.Create();

                return sharedData;
            }

            public void Dispose()
            {
                SafetyManager->Dispose();
            }

            public void Sync()
            {
                SafetyManager->BumpTemporaryHandleVersions();
            }
        }

        private SharedData m_SharedData;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();

            m_SharedData = SharedData.Create();

#if !ENABLE_UNITY_COLLECTIONS_CHECKS
            // Calling RequireForUpdate will mean that the system will not be updated if there are no dynamic bodies.
            // However, if we are performing want collider integrity checks we need the system to run regardless.
            RequireForUpdate(m_BuildPhysicsWorldSystem.DynamicEntityGroup);
#endif
        }

        protected override void OnDestroy()
        {
            m_SharedData.Dispose();
        }

        protected override void OnUpdate()
        {
            // Combine implicit input dependency with the user one
            JobHandle handle = JobHandle.CombineDependencies(Dependency, m_InputDependency);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            handle = CheckIntegrity(handle, m_BuildPhysicsWorldSystem.IntegrityCheckMap);
#endif

            ref PhysicsWorld world = ref m_BuildPhysicsWorldSystem.PhysicsWorld;

            var positionType = GetComponentTypeHandle<Translation>();
            var rotationType = GetComponentTypeHandle<Rotation>();
            var velocityType = GetComponentTypeHandle<PhysicsVelocity>();

            handle = new ExportDynamicBodiesJob
            {
                MotionVelocities = world.MotionVelocities,
                MotionDatas = world.MotionDatas,

                PositionType = positionType,
                RotationType = rotationType,
                VelocityType = velocityType
            }.ScheduleParallel(m_BuildPhysicsWorldSystem.DynamicEntityGroup, 1, handle);

            // Sync shared data.
            m_SharedData.Sync();

            int numCollisionWorldProxies = m_BuildPhysicsWorldSystem.CollisionWorldProxyGroup.CalculateEntityCount();
            if (numCollisionWorldProxies > 0)
            {
                handle = new CopyCollisionWorld
                {
                    World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                    SharedData = m_SharedData,
                    ProxyType = GetComponentTypeHandle<CollisionWorldProxy>()
                }.ScheduleParallel(m_BuildPhysicsWorldSystem.CollisionWorldProxyGroup, 1, handle);
            }

            m_OutputDependency = handle;

            // Combine implicit output dependency with user one
            Dependency = JobHandle.CombineDependencies(m_OutputDependency, Dependency);

            // Inform next system in the pipeline of its dependency
            m_EndFramePhysicsSystem.AddInputDependency(m_OutputDependency);

            // Invalidate input dependency since it's been used now
            m_InputDependency = default;
        }

        public void AddInputDependency(JobHandle inputDep)
        {
            m_InputDependency = JobHandle.CombineDependencies(m_InputDependency, inputDep);
        }

        public JobHandle GetOutputDependency()
        {
            return m_OutputDependency;
        }

        [BurstCompile]
        internal struct ExportDynamicBodiesJob : IJobEntityBatchWithIndex
        {
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public NativeArray<MotionData> MotionDatas;

            public ComponentTypeHandle<Translation> PositionType;
            public ComponentTypeHandle<Rotation> RotationType;
            public ComponentTypeHandle<PhysicsVelocity> VelocityType;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int entityStartIndex)
            {
                var chunkPositions = batchInChunk.GetNativeArray(PositionType);
                var chunkRotations = batchInChunk.GetNativeArray(RotationType);
                var chunkVelocities = batchInChunk.GetNativeArray(VelocityType);

                int numItems = batchInChunk.Count;

                for (int i = 0, motionIndex = entityStartIndex; i < numItems; i++, motionIndex++)
                {
                    MotionData md = MotionDatas[motionIndex];
                    RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
                    chunkPositions[i] = new Translation { Value = worldFromBody.pos };
                    chunkRotations[i] = new Rotation { Value = worldFromBody.rot };
                    chunkVelocities[i] = new PhysicsVelocity
                    {
                        Linear = MotionVelocities[motionIndex].LinearVelocity,
                        Angular = MotionVelocities[motionIndex].AngularVelocity
                    };
                }
            }
        }

        [BurstCompile]
        internal unsafe struct CopyCollisionWorld : IJobEntityBatch
        {
            [ReadOnly] public PhysicsWorld World;
            public SharedData SharedData;
            public ComponentTypeHandle<CollisionWorldProxy> ProxyType;

            //public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                NativeArray<CollisionWorldProxy> chunkProxies = batchInChunk.GetNativeArray(ProxyType);

                var proxy = new CollisionWorldProxy(World.CollisionWorld, SharedData.SafetyManager);

                for (var i = 0; i < batchInChunk.Count; ++i)
                    chunkProxies[i] = proxy;
            }
        }

        #region Integrity checks

        internal JobHandle CheckIntegrity(JobHandle inputDeps, NativeHashMap<uint, long> integrityCheckMap)
        {
            var positionType = GetComponentTypeHandle<Translation>(true);
            var rotationType = GetComponentTypeHandle<Rotation>(true);
            var physicsColliderType = GetComponentTypeHandle<PhysicsCollider>(true);
            var physicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true);

            var checkDynamicBodyIntegrity = new CheckDynamicBodyIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PositionType = positionType,
                RotationType = rotationType,
                PhysicsVelocityType = physicsVelocityType,
                PhysicsColliderType = physicsColliderType
            };

            inputDeps = checkDynamicBodyIntegrity.Schedule(m_BuildPhysicsWorldSystem.DynamicEntityGroup, inputDeps);

            var checkStaticBodyColliderIntegrity = new CheckColliderIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PhysicsColliderType = physicsColliderType
            };

            inputDeps = checkStaticBodyColliderIntegrity.Schedule(m_BuildPhysicsWorldSystem.StaticEntityGroup, inputDeps);

            var checkTotalIntegrity = new CheckTotalIntegrity
            {
                IntegrityCheckMap = integrityCheckMap
            };

            return checkTotalIntegrity.Schedule(inputDeps);
        }

        [BurstCompile]
        internal struct CheckDynamicBodyIntegrity : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
            [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public NativeHashMap<uint, long> IntegrityCheckMap;

            internal static void DecrementIfExists(NativeHashMap<uint, long> integrityCheckMap, uint systemVersion)
            {
                if (integrityCheckMap.TryGetValue(systemVersion, out long occurences))
                {
                    integrityCheckMap.Remove(systemVersion);
                    integrityCheckMap.Add(systemVersion, occurences - 1);
                }
            }

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                DecrementIfExists(IntegrityCheckMap, batchInChunk.GetOrderVersion());
                DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsVelocityType));
                if (batchInChunk.Has(PositionType))
                {
                    DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(PositionType));
                }
                if (batchInChunk.Has(RotationType))
                {
                    DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(RotationType));
                }
                if (batchInChunk.Has(PhysicsColliderType))
                {
                    DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsColliderType));

                    var colliders = batchInChunk.GetNativeArray(PhysicsColliderType);
                    CheckColliderFilterIntegrity(colliders);
                }
            }
        }

        [BurstCompile]
        internal struct CheckColliderIntegrity : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public NativeHashMap<uint, long> IntegrityCheckMap;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                if (batchInChunk.Has(PhysicsColliderType))
                {
                    CheckDynamicBodyIntegrity.DecrementIfExists(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsColliderType));

                    var colliders = batchInChunk.GetNativeArray(PhysicsColliderType);
                    CheckColliderFilterIntegrity(colliders);
                }
            }
        }

        [BurstCompile]
        internal struct CheckTotalIntegrity : IJob
        {
            public NativeHashMap<uint, long> IntegrityCheckMap;
            public void Execute()
            {
                var values = IntegrityCheckMap.GetValueArray(Allocator.Temp);
                var validIntegrity = true;
                for (int i = 0; i < values.Length; i++)
                {
                    if (values[i] != 0)
                    {
                        validIntegrity = false;
                        break;
                    }
                }
                if (!validIntegrity)
                {
                    SafetyChecks.ThrowInvalidOperationException("Adding/removing components or changing position/rotation/velocity/collider ECS data" +
                        " on dynamic entities during physics step");
                }
            }
        }

        // Verifies combined collision filter of compound colliders
        // ToDo: add the same for mesh once per-triangle filters are supported
        private static void CheckColliderFilterIntegrity(NativeArray<PhysicsCollider> colliders)
        {
            for (int i = 0; i < colliders.Length; i++)
            {
                var collider = colliders[i];
                if (collider.Value.Value.Type == ColliderType.Compound)
                {
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)collider.ColliderPtr;

                        var rootFilter = compoundCollider->Filter;
                        var combinedFilter = CollisionFilter.Zero;
                        for (int childIndex = 0; childIndex < compoundCollider->Children.Length; childIndex++)
                        {
                            ref CompoundCollider.Child c = ref compoundCollider->Children[childIndex];
                            combinedFilter = CollisionFilter.CreateUnion(combinedFilter, c.Collider->Filter);
                        }

                        // Check that the combined filter of all children is the same as root filter.
                        // If not, it means user has forgotten to call RefreshCollisionFilter() on the CompoundCollider.
                        if (!rootFilter.Equals(combinedFilter))
                        {
                            SafetyChecks.ThrowInvalidOperationException("CollisionFilter of a compound collider is not a union of its children. " +
                                "You must call CompoundCollider.RefreshCollisionFilter() to update the root filter after changing child filters.");
                        }
                    }
                }
            }
        }

        #endregion
    }
}
