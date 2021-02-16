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
            }.Schedule(m_BuildPhysicsWorldSystem.DynamicEntityGroup, handle);

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
                }.Schedule(m_BuildPhysicsWorldSystem.CollisionWorldProxyGroup, handle);
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
        internal struct ExportDynamicBodiesJob : IJobChunk
        {
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public NativeArray<MotionData> MotionDatas;

            public ComponentTypeHandle<Translation> PositionType;
            public ComponentTypeHandle<Rotation> RotationType;
            public ComponentTypeHandle<PhysicsVelocity> VelocityType;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int entityStartIndex)
            {
                var chunkPositions = chunk.GetNativeArray(PositionType);
                var chunkRotations = chunk.GetNativeArray(RotationType);
                var chunkVelocities = chunk.GetNativeArray(VelocityType);

                int numItems = chunk.Count;
                for(int i = 0, motionIndex = entityStartIndex; i < numItems; i++, motionIndex++)
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
        internal unsafe struct CopyCollisionWorld : IJobChunk
        {
            [ReadOnly] public PhysicsWorld World;
            public SharedData SharedData;
            public ComponentTypeHandle<CollisionWorldProxy> ProxyType;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            {
                NativeArray<CollisionWorldProxy> chunkProxies = chunk.GetNativeArray(ProxyType);

                var proxy = new CollisionWorldProxy(World.CollisionWorld, SharedData.SafetyManager);

                for (var i = 0; i < chunk.Count; ++i)
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
                PhysicsVelocityType = physicsVelocityType
            };

            inputDeps = checkDynamicBodyIntegrity.ScheduleSingle(m_BuildPhysicsWorldSystem.DynamicEntityGroup, inputDeps);

            var checkColliderIntegrity = new CheckColliderIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PhysicsColliderType = physicsColliderType
            };

            inputDeps = checkColliderIntegrity.ScheduleSingle(GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsCollider)
                }
            }), inputDeps);

            var checkTotalIntegrity = new CheckTotalIntegrity
            {
                IntegrityCheckMap = integrityCheckMap
            };

            return checkTotalIntegrity.Schedule(inputDeps);
        }

        [BurstCompile]
        internal struct CheckDynamicBodyIntegrity : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
            [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            public NativeHashMap<uint, long> IntegrityCheckMap;

            internal static void DecrementIfExists(NativeHashMap<uint, long> integrityCheckMap, uint systemVersion)
            {
                if (integrityCheckMap.TryGetValue(systemVersion, out long occurences))
                {
                    integrityCheckMap.Remove(systemVersion);
                    integrityCheckMap.Add(systemVersion, occurences - 1);
                }
            }

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            {
                DecrementIfExists(IntegrityCheckMap, chunk.GetOrderVersion());
                DecrementIfExists(IntegrityCheckMap, chunk.GetChangeVersion(PhysicsVelocityType));
                if (chunk.Has(PositionType))
                {
                    DecrementIfExists(IntegrityCheckMap, chunk.GetChangeVersion(PositionType));
                }
                if (chunk.Has(RotationType))
                {
                    DecrementIfExists(IntegrityCheckMap, chunk.GetChangeVersion(RotationType));
                }
            }
        }

        [BurstCompile]
        internal struct CheckColliderIntegrity : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public NativeHashMap<uint, long> IntegrityCheckMap;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            {
                CheckDynamicBodyIntegrity.DecrementIfExists(IntegrityCheckMap, chunk.GetChangeVersion(PhysicsColliderType));
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

        #endregion

    }
}
