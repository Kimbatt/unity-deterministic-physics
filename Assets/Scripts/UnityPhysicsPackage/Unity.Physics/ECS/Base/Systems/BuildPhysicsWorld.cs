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
    [AlwaysUpdateSystem]
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

            int numDynamicBodies = DynamicEntityGroup.CalculateEntityCount();
            int numStaticBodies = StaticEntityGroup.CalculateEntityCount();
            int numJoints = JointEntityGroup.CalculateEntityCount();

            int previousStaticBodyCount = PhysicsWorld.NumStaticBodies;

            // Resize the world's native arrays
            PhysicsWorld.Reset(
                numStaticBodies + 1, // +1 for the default static body
                numDynamicBodies,
                numJoints);

            if (PhysicsWorld.NumBodies == 0)
            {
                // If the static body count has changed, make sure to set the correct value to HaveStaticBodiesChanged
                if (PhysicsWorld.NumStaticBodies != previousStaticBodyCount)
                {
                    HaveStaticBodiesChanged[0] = 1;
                }
                else
                {
                    HaveStaticBodiesChanged[0] = 0;
                }

                m_OutputDependency = Dependency;

                ChainDependencies();

                // No bodies in the scene, no need to do anything else
                return;
            }

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
                    staticBodiesCheckHandle = new Jobs.CheckStaticBodyChangesJob
                    {
                        LocalToWorldType = localToWorldType,
                        ParentType = parentType,
                        PositionType = positionType,
                        RotationType = rotationType,
                        PhysicsColliderType = physicsColliderType,
                        m_LastSystemVersion = LastSystemVersion,
                        Result = HaveStaticBodiesChanged
                    }.ScheduleParallel(StaticEntityGroup, 1, Dependency);
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
                    BodyIndex = PhysicsWorld.Bodies.Length - 1,
                    EntityBodyIndexMap = PhysicsWorld.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
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
                        RigidBodies = PhysicsWorld.Bodies,
                        EntityBodyIndexMap = PhysicsWorld.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                    }.ScheduleParallel(DynamicEntityGroup, 1, Dependency));

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
                    }.ScheduleParallel(DynamicEntityGroup, 1, Dependency));
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
                        RigidBodies = PhysicsWorld.Bodies,
                        EntityBodyIndexMap = PhysicsWorld.CollisionWorld.EntityBodyIndexMap.AsParallelWriter(),
                    }.ScheduleParallel(StaticEntityGroup, 1, Dependency));
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
                        NumDynamicBodies = numDynamicBodies,
                        EntityBodyIndexMap = PhysicsWorld.CollisionWorld.EntityBodyIndexMap,
                        EntityJointIndexMap = PhysicsWorld.DynamicsWorld.EntityJointIndexMap.AsParallelWriter(),
                    }.ScheduleParallel(JointEntityGroup, 1, handle));
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
                    HaveStaticBodiesChanged, handle, stepComponent.MultiThreaded > 0);
                jobHandles.Add(buildBroadphaseHandle);

                m_OutputDependency = JobHandle.CombineDependencies(jobHandles);
            }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            RecordIntegrity(IntegrityCheckMap);
#endif

            ChainDependencies();
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

        private void ChainDependencies()
        {
            // Combine implicit output dependency with user one
            Dependency = JobHandle.CombineDependencies(m_OutputDependency, Dependency);

            // Inform next system in the pipeline of its dependency
            m_StepPhysicsWorldSystem.AddInputDependency(Dependency);

            // Invalidate input dependencies since they've been used by now
            m_InputDependency = default;
            m_InputDependencyToComplete = default;
        }

        #region Jobs

        private static class Jobs
        {
            [BurstCompile]
            internal struct CheckStaticBodyChangesJob : IJobEntityBatch
            {
                [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
                [ReadOnly] public ComponentTypeHandle<Parent> ParentType;
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;

                [NativeDisableParallelForRestriction]
                public NativeArray<int> Result;

                public uint m_LastSystemVersion;

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
                {
                    bool didBatchChange =
                        batchInChunk.DidChange(LocalToWorldType, m_LastSystemVersion) ||
                        batchInChunk.DidChange(PositionType, m_LastSystemVersion) ||
                        batchInChunk.DidChange(RotationType, m_LastSystemVersion) ||
                        batchInChunk.DidChange(PhysicsColliderType, m_LastSystemVersion) ||
                        batchInChunk.DidOrderChange(m_LastSystemVersion);
                    if (didBatchChange)
                    {
                        // Note that multiple worker threads may be running at the same time.
                        // They either write 1 to Result[0] or not write at all.  In case multiple
                        // threads are writing 1 to this variable, in C#, reads or writes of int
                        // data type are atomic, which guarantees that Result[0] is 1.
                        Result[0] = 1;
                    }
                }
            }

            [BurstCompile]
            internal struct CreateDefaultStaticRigidBody : IJob
            {
                [NativeDisableContainerSafetyRestriction]
                public NativeArray<RigidBody> NativeBodies;
                public int BodyIndex;

                [NativeDisableContainerSafetyRestriction]
                public NativeHashMap<Entity, int>.ParallelWriter EntityBodyIndexMap;

                public void Execute()
                {
                    NativeBodies[BodyIndex] = new RigidBody
                    {
                        WorldFromBody = new RigidTransform(quaternion.identity, float3.zero),
                        Collider = default,
                        Entity = Entity.Null,
                        CustomTags = 0
                    };
                    EntityBodyIndexMap.TryAdd(Entity.Null, BodyIndex);
                }
            }

            [BurstCompile]
            internal struct CreateRigidBodies : IJobEntityBatchWithIndex
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
                [NativeDisableContainerSafetyRestriction] public NativeHashMap<Entity, int>.ParallelWriter EntityBodyIndexMap;

                //public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
                public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int firstEntityIndex)
                {
                    NativeArray<PhysicsCollider> chunkColliders = batchInChunk.GetNativeArray(PhysicsColliderType);
                    NativeArray<LocalToWorld> chunkLocalToWorlds = batchInChunk.GetNativeArray(LocalToWorldType);
                    NativeArray<Translation> chunkPositions = batchInChunk.GetNativeArray(PositionType);
                    NativeArray<Rotation> chunkRotations = batchInChunk.GetNativeArray(RotationType);
                    NativeArray<Entity> chunkEntities = batchInChunk.GetNativeArray(EntityType);
                    NativeArray<PhysicsCustomTags> chunkCustomTags = batchInChunk.GetNativeArray(PhysicsCustomTagsType);

                    int instanceCount = batchInChunk.Count;
                    int rbIndex = FirstBodyIndex + firstEntityIndex;

                    bool hasChunkPhysicsColliderType = batchInChunk.Has(PhysicsColliderType);
                    bool hasChunkPhysicsCustomTagsType = batchInChunk.Has(PhysicsCustomTagsType);
                    bool hasChunkParentType = batchInChunk.Has(ParentType);
                    bool hasChunkLocalToWorldType = batchInChunk.Has(LocalToWorldType);
                    bool hasChunkPositionType = batchInChunk.Has(PositionType);
                    bool hasChunkRotationType = batchInChunk.Has(RotationType);

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
                        EntityBodyIndexMap.TryAdd(chunkEntities[i], rbIndex);
                    }
                }
            }

            [BurstCompile]
            internal struct CreateMotions : IJobEntityBatchWithIndex
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

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int firstEntityIndex)
                {
                    NativeArray<Translation> chunkPositions = batchInChunk.GetNativeArray(PositionType);
                    NativeArray<Rotation> chunkRotations = batchInChunk.GetNativeArray(RotationType);
                    NativeArray<PhysicsVelocity> chunkVelocities = batchInChunk.GetNativeArray(PhysicsVelocityType);
                    NativeArray<PhysicsMass> chunkMasses = batchInChunk.GetNativeArray(PhysicsMassType);
                    NativeArray<PhysicsMassOverride> chunkMassOverrides = batchInChunk.GetNativeArray(PhysicsMassOverrideType);
                    NativeArray<PhysicsDamping> chunkDampings = batchInChunk.GetNativeArray(PhysicsDampingType);
                    NativeArray<PhysicsGravityFactor> chunkGravityFactors = batchInChunk.GetNativeArray(PhysicsGravityFactorType);

                    int motionStart = firstEntityIndex;
                    int instanceCount = batchInChunk.Count;

                    bool hasChunkPhysicsGravityFactorType = batchInChunk.Has(PhysicsGravityFactorType);
                    bool hasChunkPhysicsDampingType = batchInChunk.Has(PhysicsDampingType);
                    bool hasChunkPhysicsMassType = batchInChunk.Has(PhysicsMassType);
                    bool hasChunkPhysicsMassOverrideType = batchInChunk.Has(PhysicsMassOverrideType);

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
            internal struct CreateJoints : IJobEntityBatchWithIndex
            {
                [ReadOnly] public ComponentTypeHandle<PhysicsConstrainedBodyPair> ConstrainedBodyPairComponentType;
                [ReadOnly] public ComponentTypeHandle<PhysicsJoint> JointComponentType;
                [ReadOnly] public EntityTypeHandle EntityType;
                [ReadOnly] public NativeArray<RigidBody> RigidBodies;
                [ReadOnly] public int NumDynamicBodies;
                [ReadOnly] public NativeHashMap<Entity, int> EntityBodyIndexMap;

                [NativeDisableParallelForRestriction] public NativeArray<Joint> Joints;
                [NativeDisableParallelForRestriction] public NativeHashMap<Entity, int>.ParallelWriter EntityJointIndexMap;

                public int DefaultStaticBodyIndex;

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex, int firstEntityIndex)
                {
                    NativeArray<PhysicsConstrainedBodyPair> chunkBodyPair = batchInChunk.GetNativeArray(ConstrainedBodyPairComponentType);
                    NativeArray<PhysicsJoint> chunkJoint = batchInChunk.GetNativeArray(JointComponentType);
                    NativeArray<Entity> chunkEntities = batchInChunk.GetNativeArray(EntityType);

                    int instanceCount = batchInChunk.Count;
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
                            BodyIndexB = entityB == Entity.Null ? DefaultStaticBodyIndex : -1,
                        };

                        // Find the body indices
                        pair.BodyIndexA = EntityBodyIndexMap.TryGetValue(entityA, out var idxA) ? idxA : -1;
                        pair.BodyIndexB = EntityBodyIndexMap.TryGetValue(entityB, out var idxB) ? idxB : -1;

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
                        EntityJointIndexMap.TryAdd(chunkEntities[i], firstEntityIndex + i);
                    }
                }
            }

            #region Integrity checks

            [BurstCompile]
            internal struct RecordDynamicBodyIntegrity : IJobEntityBatch
            {
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;

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

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
                {
                    AddOrIncrement(IntegrityCheckMap, batchInChunk.GetOrderVersion());
                    AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsVelocityType));
                    if (batchInChunk.Has(PositionType))
                    {
                        AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(PositionType));
                    }
                    if (batchInChunk.Has(RotationType))
                    {
                        AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(RotationType));
                    }
                    if (batchInChunk.Has(PhysicsColliderType))
                    {
                        AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsColliderType));
                    }
                }
            }

            [BurstCompile]
            internal struct RecordColliderIntegrity : IJobEntityBatch
            {
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
                public NativeHashMap<uint, long> IntegrityCheckMap;

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
                {
                    if (batchInChunk.Has(PhysicsColliderType))
                    {
                        RecordDynamicBodyIntegrity.AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsColliderType));
                    }
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
                PhysicsVelocityType = physicsVelocityType,
                PhysicsColliderType = physicsColliderType
            };

            m_OutputDependency = dynamicBodyIntegrity.Schedule(DynamicEntityGroup, m_OutputDependency);

            var staticBodyColliderIntegrity = new Jobs.RecordColliderIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PhysicsColliderType = physicsColliderType
            };

            m_OutputDependency = staticBodyColliderIntegrity.Schedule(StaticEntityGroup, m_OutputDependency);
        }

        #endregion
    }
}
