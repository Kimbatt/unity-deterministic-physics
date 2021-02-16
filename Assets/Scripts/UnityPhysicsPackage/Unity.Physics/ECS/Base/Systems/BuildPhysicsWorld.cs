using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;
using UnityS.Transforms;
using UnityEngine.Assertions;

namespace UnityS.Physics.Systems
{
    // A system which builds the physics world based on the entity world.
    // The world will contain a rigid body for every entity which has a rigid body component,
    // and a joint for every entity which has a joint component.
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public class BuildPhysicsWorld : SystemBase, IPhysicsSystem
    {
        private JobHandle m_InputDependencyToComplete;
        private JobHandle m_InputDependency;
        private JobHandle m_OutputDependency;

        public PhysicsWorld PhysicsWorld = new PhysicsWorld(0, 0, 0);

        // Entity group queries
        public EntityQuery DynamicEntityGroup { get; private set; }
        public EntityQuery StaticEntityGroup { get; private set; }
        public EntityQuery JointEntityGroup { get; private set; }

        public EntityQuery CollisionWorldProxyGroup { get; private set; }

        internal NativeArray<int> HaveStaticBodiesChanged = new NativeArray<int>(1, Allocator.Persistent);

        StepPhysicsWorld m_StepPhysicsWorldSystem;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        internal NativeHashMap<uint, long> IntegrityCheckMap = new NativeHashMap<uint, long>(4, Allocator.Persistent);
#endif

        protected override void OnCreate()
        {
            base.OnCreate();

            DynamicEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity),
                    typeof(Translation),
                    typeof(Rotation)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsExclude)
                }
            });

            StaticEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsCollider)
                },
                Any = new ComponentType[]
                {
                    typeof(LocalToWorld),
                    typeof(Translation),
                    typeof(Rotation)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsExclude),
                    typeof(PhysicsVelocity)
                }
            });

            JointEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsConstrainedBodyPair),
                    typeof(PhysicsJoint)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsExclude)
                }
            });

            CollisionWorldProxyGroup = GetEntityQuery(ComponentType.ReadWrite<CollisionWorldProxy>());

            m_StepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorld>();
        }

        protected override void OnDestroy()
        {
            PhysicsWorld.Dispose();
            HaveStaticBodiesChanged.Dispose();

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            IntegrityCheckMap.Dispose();
#endif

            base.OnDestroy();
        }

        protected override void OnUpdate()
        {
            // Make sure last frame's physics jobs are complete before any new ones start
            m_InputDependencyToComplete.Complete();

            // Combine implicit input dependency with the user one
            Dependency = JobHandle.CombineDependencies(Dependency, m_InputDependency);

            // Extract types used by initialize jobs
            var entityType = GetEntityTypeHandle();
            var localToWorldType = GetComponentTypeHandle<LocalToWorld>(true);
            var parentType = GetComponentTypeHandle<Parent>(true);
            var positionType = GetComponentTypeHandle<Translation>(true);
            var rotationType = GetComponentTypeHandle<Rotation>(true);
            var physicsColliderType = GetComponentTypeHandle<PhysicsCollider>(true);
            var physicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true);
            var physicsMassType = GetComponentTypeHandle<PhysicsMass>(true);
            var physicsMassOverrideType = GetComponentTypeHandle<PhysicsMassOverride>(true);
            var physicsDampingType = GetComponentTypeHandle<PhysicsDamping>(true);
            var physicsGravityFactorType = GetComponentTypeHandle<PhysicsGravityFactor>(true);
            var physicsCustomTagsType = GetComponentTypeHandle<PhysicsCustomTags>(true);
            var physicsConstrainedBodyPairType = GetComponentTypeHandle<PhysicsConstrainedBodyPair>(true);
            var physicsJointType = GetComponentTypeHandle<PhysicsJoint>(true);

            int numDynamicBodies = DynamicEntityGroup.CalculateEntityCount();
            int numStaticBodies = StaticEntityGroup.CalculateEntityCount();
            int numJoints = JointEntityGroup.CalculateEntityCount();

            int previousStaticBodyCount = PhysicsWorld.NumStaticBodies;

            // Resize the world's native arrays
            PhysicsWorld.Reset(
                numStaticBodies + 1, // +1 for the default static body
                numDynamicBodies,
                numJoints);

            // Determine if the static bodies have changed in any way that will require the static broadphase tree to be rebuilt
            JobHandle staticBodiesCheckHandle = default;

            HaveStaticBodiesChanged[0] = 0;
            {
                if (PhysicsWorld.NumStaticBodies != previousStaticBodyCount)
                {
                    HaveStaticBodiesChanged[0] = 1;
                }
                else
                {
                    // Make a job to test for changes
                    int numChunks;
                    using (NativeArray<ArchetypeChunk> chunks = StaticEntityGroup.CreateArchetypeChunkArray(Allocator.TempJob))
                    {
                        numChunks = chunks.Length;
                    }
                    var chunksHaveChanges = new NativeArray<int>(numChunks, Allocator.TempJob);

                    staticBodiesCheckHandle = new Jobs.CheckStaticBodyChangesJob
                    {
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        ChunkHasChangesOutput = chunksHaveChanges,
                        m_LastSystemVersion = LastSystemVersion
                    }.Schedule(StaticEntityGroup, Dependency);

                    staticBodiesCheckHandle = new Jobs.CheckStaticBodyChangesReduceJob
                    {
                        ChunkHasChangesOutput = chunksHaveChanges,
                        Result = HaveStaticBodiesChanged
                    }.Schedule(staticBodiesCheckHandle);
                }
            }

            using (var jobHandles = new NativeList<JobHandle>(4, Allocator.Temp))
            {
                // Static body changes check jobs
                jobHandles.Add(staticBodiesCheckHandle);

                // Create the default static body at the end of the body list
                // TODO: could skip this if no joints present
                jobHandles.Add(new Jobs.CreateDefaultStaticRigidBody
                {
                    NativeBodies = PhysicsWorld.Bodies,
                    BodyIndex = PhysicsWorld.Bodies.Length - 1
                }.Schedule(Dependency));

                // Dynamic bodies.
                // Create these separately from static bodies to maintain a 1:1 mapping
                // between dynamic bodies and their motions.
                if (numDynamicBodies > 0)
                {
                    jobHandles.Add(new Jobs.CreateRigidBodies
                    {
                        EntityType = entityType,
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        PhysicsCustomTagsType = physicsCustomTagsType,

                        FirstBodyIndex = 0,
                        RigidBodies = PhysicsWorld.Bodies
                    }.Schedule(DynamicEntityGroup, Dependency));

                    jobHandles.Add(new Jobs.CreateMotions
                    {
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsVelocityType = physicsVelocityType,
                        PhysicsMassType = physicsMassType,
                        PhysicsMassOverrideType = physicsMassOverrideType,
                        PhysicsDampingType = physicsDampingType,
                        PhysicsGravityFactorType = physicsGravityFactorType,

                        MotionDatas = PhysicsWorld.MotionDatas,
                        MotionVelocities = PhysicsWorld.MotionVelocities
                    }.Schedule(DynamicEntityGroup, Dependency));
                }

                // Now, schedule creation of static bodies, with FirstBodyIndex pointing after
                // the dynamic and kinematic bodies
                if (numStaticBodies > 0)
                {
                    jobHandles.Add(new Jobs.CreateRigidBodies
                    {
                        EntityType = entityType,
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        PhysicsCustomTagsType = physicsCustomTagsType,

                        FirstBodyIndex = numDynamicBodies,
                        RigidBodies = PhysicsWorld.Bodies
                    }.Schedule(StaticEntityGroup, Dependency));
                }

                var handle = JobHandle.CombineDependencies(jobHandles);
                jobHandles.Clear();

                // Build joints
                if (numJoints > 0)
                {
                    jobHandles.Add(new Jobs.CreateJoints
                    {
                        ConstrainedBodyPairComponentType = physicsConstrainedBodyPairType,
                        JointComponentType = physicsJointType,
                        EntityType = entityType,
                        RigidBodies = PhysicsWorld.Bodies,
                        Joints = PhysicsWorld.Joints,
                        DefaultStaticBodyIndex = PhysicsWorld.Bodies.Length - 1,
                        NumDynamicBodies = numDynamicBodies
                    }.Schedule(JointEntityGroup, handle));
                }

                // Build the broadphase
                // TODO: could optimize this by gathering the AABBs and filters at the same time as building the bodies above
                sfloat timeStep = (sfloat)Time.DeltaTime;

                PhysicsStep stepComponent = PhysicsStep.Default;
                if (HasSingleton<PhysicsStep>())
                {
                    stepComponent = GetSingleton<PhysicsStep>();
                }

                JobHandle buildBroadphaseHandle = PhysicsWorld.CollisionWorld.ScheduleBuildBroadphaseJobs(
                    ref PhysicsWorld, timeStep, stepComponent.Gravity,
                    HaveStaticBodiesChanged, handle, stepComponent.ThreadCountHint);
                jobHandles.Add(buildBroadphaseHandle);

                m_OutputDependency = JobHandle.CombineDependencies(jobHandles);
            }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            RecordIntegrity(IntegrityCheckMap);
#endif

            // Combine implicit output dependency with user one
            Dependency = JobHandle.CombineDependencies(m_OutputDependency, Dependency);

            // Inform next system in the pipeline of its dependency
            m_StepPhysicsWorldSystem.AddInputDependency(Dependency);

            // Invalidate input dependencies since they've been used by now
            m_InputDependency = default;
            m_InputDependencyToComplete = default;
        }

        public void AddInputDependencyToComplete(JobHandle dependencyToComplete)
        {
            m_InputDependencyToComplete = JobHandle.CombineDependencies(m_InputDependencyToComplete, dependencyToComplete);
        }

        public void AddInputDependency(JobHandle inputDep)
        {
            m_InputDependency = JobHandle.CombineDependencies(m_InputDependency, inputDep);
        }

        public JobHandle GetOutputDependency()
        {
            return m_OutputDependency;
        }

        #region Jobs

        private static class Jobs
        {
            [BurstCompile]
            internal struct CheckStaticBodyChangesJob : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ComponentTypeHandle<Parent> ParentType;
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;

                [NativeDisableParallelForRestriction] public NativeArray<int> ChunkHasChangesOutput;
                public uint m_LastSystemVersion;

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    bool didChunkChange =
                        chunk.DidChange(LocalToWorldType, m_LastSystemVersion) ||
                        chunk.DidChange(PositionType, m_LastSystemVersion) ||
                        chunk.DidChange(RotationType, m_LastSystemVersion) ||
                        chunk.DidChange(PhysicsColliderType, m_LastSystemVersion) ||
                        chunk.DidOrderChange(m_LastSystemVersion);
                    ChunkHasChangesOutput[chunkIndex] = didChunkChange ? 1 : 0;
                }
            }

            [BurstCompile]
            internal struct CheckStaticBodyChangesReduceJob : IJob
            {
                [ReadOnly][DeallocateOnJobCompletion] public NativeArray<int> ChunkHasChangesOutput;
                public NativeArray<int> Result;

                public void Execute()
                {
                    for (int i = 0; i < ChunkHasChangesOutput.Length; i++)
                    {
                        if (ChunkHasChangesOutput[i] > 0)
                        {
                            Result[0] = 1;
                            return;
                        }
                    }

                    Result[0] = 0;
                }
            }

            [BurstCompile]
            internal struct CreateDefaultStaticRigidBody : IJob
            {
                [NativeDisableContainerSafetyRestriction]
                public NativeArray<RigidBody> NativeBodies;
                public int BodyIndex;

                public void Execute()
                {
                    NativeBodies[BodyIndex] = new RigidBody
                    {
                        WorldFromBody = new RigidTransform(quaternion.identity, float3.zero),
                        Collider = default,
                        Entity = Entity.Null,
                        CustomTags = 0
                    };
                }
            }

            [BurstCompile]
            internal struct CreateRigidBodies : IJobChunk
            {
                [ReadOnly] public EntityTypeHandle EntityType;
                [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ComponentTypeHandle<Parent> ParentType;
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCustomTags> PhysicsCustomTagsType;
                [ReadOnly] public int FirstBodyIndex;

                [NativeDisableContainerSafetyRestriction] public NativeArray<RigidBody> RigidBodies;

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    NativeArray<PhysicsCollider> chunkColliders = chunk.GetNativeArray(PhysicsColliderType);
                    NativeArray<LocalToWorld> chunkLocalToWorlds = chunk.GetNativeArray(LocalToWorldType);
                    NativeArray<Translation> chunkPositions = chunk.GetNativeArray(PositionType);
                    NativeArray<Rotation> chunkRotations = chunk.GetNativeArray(RotationType);
                    NativeArray<Entity> chunkEntities = chunk.GetNativeArray(EntityType);
                    NativeArray<PhysicsCustomTags> chunkCustomTags = chunk.GetNativeArray(PhysicsCustomTagsType);

                    int instanceCount = chunk.Count;
                    int rbIndex = FirstBodyIndex + firstEntityIndex;

                    bool hasChunkPhysicsColliderType = chunk.Has(PhysicsColliderType);
                    bool hasChunkPhysicsCustomTagsType = chunk.Has(PhysicsCustomTagsType);
                    bool hasChunkParentType = chunk.Has(ParentType);
                    bool hasChunkLocalToWorldType = chunk.Has(LocalToWorldType);
                    bool hasChunkPositionType = chunk.Has(PositionType);
                    bool hasChunkRotationType = chunk.Has(RotationType);

                    RigidTransform worldFromBody = RigidTransform.identity;
                    for (int i = 0; i < instanceCount; i++, rbIndex++)
                    {
                        // if entities are in a transform hierarchy then Translation/Rotation are in the space of their parents
                        // in that case, LocalToWorld is the only common denominator for world space
                        if (hasChunkParentType)
                        {
                            if (hasChunkLocalToWorldType)
                            {
                                var localToWorld = chunkLocalToWorlds[i];
                                worldFromBody = Math.DecomposeRigidBodyTransform(localToWorld.Value);
                            }
                        }
                        else
                        {
                            if (hasChunkPositionType)
                            {
                                worldFromBody.pos = chunkPositions[i].Value;
                            }
                            else if (hasChunkLocalToWorldType)
                            {
                                worldFromBody.pos = chunkLocalToWorlds[i].Position;
                            }

                            if (hasChunkRotationType)
                            {
                                worldFromBody.rot = chunkRotations[i].Value;
                            }
                            else if (hasChunkLocalToWorldType)
                            {
                                var localToWorld = chunkLocalToWorlds[i];
                                worldFromBody.rot = Math.DecomposeRigidBodyOrientation(localToWorld.Value);
                            }
                        }

                        RigidBodies[rbIndex] = new RigidBody
                        {
                            WorldFromBody = new RigidTransform(worldFromBody.rot, worldFromBody.pos),
                            Collider = hasChunkPhysicsColliderType ? chunkColliders[i].Value : default,
                            Entity = chunkEntities[i],
                            CustomTags = hasChunkPhysicsCustomTagsType ? chunkCustomTags[i].Value : (byte)0
                        };
                    }
                }
            }

            [BurstCompile]
            internal struct CreateMotions : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
                [ReadOnly] public ComponentTypeHandle<PhysicsMass> PhysicsMassType;
                [ReadOnly] public ComponentTypeHandle<PhysicsMassOverride> PhysicsMassOverrideType;
                [ReadOnly] public ComponentTypeHandle<PhysicsDamping> PhysicsDampingType;
                [ReadOnly] public ComponentTypeHandle<PhysicsGravityFactor> PhysicsGravityFactorType;

                [NativeDisableParallelForRestriction] public NativeArray<MotionData> MotionDatas;
                [NativeDisableParallelForRestriction] public NativeArray<MotionVelocity> MotionVelocities;

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    NativeArray<Translation> chunkPositions = chunk.GetNativeArray(PositionType);
                    NativeArray<Rotation> chunkRotations = chunk.GetNativeArray(RotationType);
                    NativeArray<PhysicsVelocity> chunkVelocities = chunk.GetNativeArray(PhysicsVelocityType);
                    NativeArray<PhysicsMass> chunkMasses = chunk.GetNativeArray(PhysicsMassType);
                    NativeArray<PhysicsMassOverride> chunkMassOverrides = chunk.GetNativeArray(PhysicsMassOverrideType);
                    NativeArray<PhysicsDamping> chunkDampings = chunk.GetNativeArray(PhysicsDampingType);
                    NativeArray<PhysicsGravityFactor> chunkGravityFactors = chunk.GetNativeArray(PhysicsGravityFactorType);

                    int motionStart = firstEntityIndex;
                    int instanceCount = chunk.Count;

                    bool hasChunkPhysicsGravityFactorType = chunk.Has(PhysicsGravityFactorType);
                    bool hasChunkPhysicsDampingType = chunk.Has(PhysicsDampingType);
                    bool hasChunkPhysicsMassType = chunk.Has(PhysicsMassType);
                    bool hasChunkPhysicsMassOverrideType = chunk.Has(PhysicsMassOverrideType);

                    // Note: Transform and AngularExpansionFactor could be calculated from PhysicsCollider.MassProperties
                    // However, to avoid the cost of accessing the collider we assume an infinite mass at the origin of a ~1m^3 box.
                    // For better performance with spheres, or better behavior for larger and/or more irregular colliders
                    // you should add a PhysicsMass component to get the true values
                    var defaultPhysicsMass = new PhysicsMass
                    {
                        Transform = RigidTransform.identity,
                        InverseMass = sfloat.Zero,
                        InverseInertia = float3.zero,
                        AngularExpansionFactor = sfloat.One,
                    };

                    // Note: if a dynamic body infinite mass then assume no gravity should be applied
                    sfloat defaultGravityFactor = hasChunkPhysicsMassType ? sfloat.One : sfloat.Zero;

                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        var isKinematic = !hasChunkPhysicsMassType || hasChunkPhysicsMassOverrideType && chunkMassOverrides[i].IsKinematic != 0;
                        MotionVelocities[motionIndex] = new MotionVelocity
                        {
                            LinearVelocity = chunkVelocities[i].Linear,
                            AngularVelocity = chunkVelocities[i].Angular,
                            InverseInertia = isKinematic ? defaultPhysicsMass.InverseInertia : chunkMasses[i].InverseInertia,
                            InverseMass = isKinematic ? defaultPhysicsMass.InverseMass : chunkMasses[i].InverseMass,
                            AngularExpansionFactor = hasChunkPhysicsMassType ? chunkMasses[i].AngularExpansionFactor : defaultPhysicsMass.AngularExpansionFactor,
                            GravityFactor = isKinematic ? sfloat.Zero : hasChunkPhysicsGravityFactorType ? chunkGravityFactors[i].Value : defaultGravityFactor
                        };
                    }

                    // Note: these defaults assume a dynamic body with infinite mass, hence no damping
                    var defaultPhysicsDamping = new PhysicsDamping
                    {
                        Linear = sfloat.Zero,
                        Angular = sfloat.Zero,
                    };

                    // Create motion datas
                    for (int i = 0, motionIndex = motionStart; i < instanceCount; i++, motionIndex++)
                    {
                        PhysicsMass mass = hasChunkPhysicsMassType ? chunkMasses[i] : defaultPhysicsMass;
                        PhysicsDamping damping = hasChunkPhysicsDampingType ? chunkDampings[i] : defaultPhysicsDamping;

                        MotionDatas[motionIndex] = new MotionData
                        {
                            WorldFromMotion = new RigidTransform(
                                math.mul(chunkRotations[i].Value, mass.InertiaOrientation),
                                math.rotate(chunkRotations[i].Value, mass.CenterOfMass) + chunkPositions[i].Value
                                ),
                            BodyFromMotion = new RigidTransform(mass.InertiaOrientation, mass.CenterOfMass),
                            LinearDamping = damping.Linear,
                            AngularDamping = damping.Angular
                        };
                    }
                }
            }

            [BurstCompile]
            internal struct CreateJoints : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<PhysicsConstrainedBodyPair> ConstrainedBodyPairComponentType;
                [ReadOnly] public ComponentTypeHandle<PhysicsJoint> JointComponentType;
                [ReadOnly] public EntityTypeHandle EntityType;
                [ReadOnly] public NativeArray<RigidBody> RigidBodies;
                [ReadOnly] public int NumDynamicBodies;

                [NativeDisableParallelForRestriction]
                public NativeArray<Joint> Joints;

                public int DefaultStaticBodyIndex;

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    NativeArray<PhysicsConstrainedBodyPair> chunkBodyPair = chunk.GetNativeArray(ConstrainedBodyPairComponentType);
                    NativeArray<PhysicsJoint> chunkJoint = chunk.GetNativeArray(JointComponentType);
                    NativeArray<Entity> chunkEntities = chunk.GetNativeArray(EntityType);

                    int instanceCount = chunk.Count;
                    for (int i = 0; i < instanceCount; i++)
                    {
                        var bodyPair = chunkBodyPair[i];
                        var entityA = bodyPair.EntityA;
                        var entityB = bodyPair.EntityB;
                        Assert.IsTrue(entityA != entityB);

                        PhysicsJoint joint = chunkJoint[i];

                        // TODO find a reasonable way to look up the constraint body indices
                        // - stash body index in a component on the entity? But we don't have random access to Entity data in a job
                        // - make a map from entity to rigid body index? Sounds bad and I don't think there is any NativeArray-based map data structure yet

                        // If one of the entities is null, use the default static entity
                        var pair = new BodyIndexPair
                        {
                            BodyIndexA = entityA == Entity.Null ? DefaultStaticBodyIndex : -1,
                            BodyIndexB = entityB == Entity.Null ? DefaultStaticBodyIndex : -1
                        };

                        // Find the body indices
                        for (int bodyIndex = 0; bodyIndex < RigidBodies.Length; bodyIndex++)
                        {
                            if (entityA != Entity.Null)
                            {
                                if (RigidBodies[bodyIndex].Entity == entityA)
                                {
                                    pair.BodyIndexA = bodyIndex;
                                    if (pair.BodyIndexB >= 0)
                                    {
                                        break;
                                    }
                                }
                            }

                            if (entityB != Entity.Null)
                            {
                                if (RigidBodies[bodyIndex].Entity == entityB)
                                {
                                    pair.BodyIndexB = bodyIndex;
                                    if (pair.BodyIndexA >= 0)
                                    {
                                        break;
                                    }
                                }
                            }
                        }

                        bool isInvalid = false;
                        // Invalid if we have not found the body indices...
                        isInvalid |= (pair.BodyIndexA == -1 || pair.BodyIndexB == -1);
                        // ... or if we are constraining two static bodies
                        // Mark static-static invalid since they are not going to affect simulation in any way.
                        isInvalid |= (pair.BodyIndexA >= NumDynamicBodies && pair.BodyIndexB >= NumDynamicBodies);
                        if (isInvalid)
                        {
                            pair = BodyIndexPair.Invalid;
                        }

                        Joints[firstEntityIndex + i] = new Joint
                        {
                            BodyPair = pair,
                            Entity = chunkEntities[i],
                            EnableCollision = (byte)chunkBodyPair[i].EnableCollision,
                            AFromJoint = joint.BodyAFromJoint.AsMTransform(),
                            BFromJoint = joint.BodyBFromJoint.AsMTransform(),
                            Version = joint.Version,
                            Constraints = joint.GetConstraints()
                        };
                    }
                }
            }

            #region Integrity checks

            [BurstCompile]
            internal struct RecordDynamicBodyIntegrity : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;

                public NativeHashMap<uint, long> IntegrityCheckMap;

                internal static void AddOrIncrement(NativeHashMap<uint, long> integrityCheckMap, uint systemVersion)
                {
                    if (integrityCheckMap.TryGetValue(systemVersion, out long occurences))
                    {
                        integrityCheckMap.Remove(systemVersion);
                        integrityCheckMap.Add(systemVersion, occurences + 1);
                    }
                    else
                    {
                        integrityCheckMap.Add(systemVersion, 1);
                    }
                }

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    AddOrIncrement(IntegrityCheckMap, chunk.GetOrderVersion());
                    AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(PhysicsVelocityType));
                    if (chunk.Has(PositionType))
                    {
                        AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(PositionType));
                    }
                    if (chunk.Has(RotationType))
                    {
                        AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(RotationType));
                    }
                }
            }

            [BurstCompile]
            internal struct RecordColliderIntegrity : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
                public NativeHashMap<uint, long> IntegrityCheckMap;

                public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                {
                    RecordDynamicBodyIntegrity.AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(PhysicsColliderType));
                }
            }

            #endregion
        }

        #endregion

        #region Integrity checks

        internal void RecordIntegrity(NativeHashMap<uint, long> integrityCheckMap)
        {
            var positionType = GetComponentTypeHandle<Translation>(true);
            var rotationType = GetComponentTypeHandle<Rotation>(true);
            var physicsColliderType = GetComponentTypeHandle<PhysicsCollider>(true);
            var physicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true);

            integrityCheckMap.Clear();

            var dynamicBodyIntegrity = new Jobs.RecordDynamicBodyIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PositionType = positionType,
                RotationType = rotationType,
                PhysicsVelocityType = physicsVelocityType
            };

            m_OutputDependency = dynamicBodyIntegrity.ScheduleSingle(DynamicEntityGroup, m_OutputDependency);

            var colliderIntegrity = new Jobs.RecordColliderIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PhysicsColliderType = physicsColliderType
            };

            m_OutputDependency = colliderIntegrity.ScheduleSingle(GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsCollider)
                }
            }), m_OutputDependency);
        }

        #endregion
    }
}
