using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;
using UnityS.Physics.Systems;
using UnityS.Transforms;

namespace UnityS.Physics.GraphicsIntegration
{
    /// <summary>
    /// A system that can smooth out the motion of rigid bodies if the fixed physics tick rate is slower than the variable graphics framerate.
    /// Each affected body's LocalToWorld is adjusted before rendering, but its underlying Translation and Rotation values are left alone.
    /// </summary>
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateBefore(typeof(EndFrameTRSToLocalToWorldSystem))]
    public class SmoothRigidBodiesGraphicalMotion : SystemBase, IPhysicsSystem
    {
        JobHandle m_InputDependency;
        JobHandle m_OutputDependency;

        RecordMostRecentFixedTime m_RecordMostRecentFixedTime;

        /// <summary>
        /// An entity query matching dynamic rigid bodies whose motion should be smoothed.
        /// </summary>
        public EntityQuery SmoothedDynamicBodiesGroup { get; private set; }

        /// <summary>
        /// Inject an input dependency into this system's job chain.
        /// </summary>
        /// <param name="inputDep">The JobHandle for the dependency.</param>
        public void AddInputDependency(JobHandle inputDep) =>
            m_InputDependency = JobHandle.CombineDependencies(m_InputDependency, inputDep);

        /// <summary>
        /// Get the final job handle for this system;
        /// </summary>
        /// <returns></returns>
        public JobHandle GetOutputDependency() => m_OutputDependency;

        protected override void OnCreate()
        {
            base.OnCreate();
            SmoothedDynamicBodiesGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(Translation),
                    typeof(Rotation),
                    typeof(PhysicsGraphicalSmoothing),
                    typeof(LocalToWorld)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsExclude)
                }
            });
            RequireForUpdate(SmoothedDynamicBodiesGroup);
            m_RecordMostRecentFixedTime = World.GetOrCreateSystem<RecordMostRecentFixedTime>();
        }

        protected override void OnUpdate()
        {
            // Note: this is only for rendering, not affecting the physics simulation
            // So float operations can be used here, because it's not required to be deterministic
            sfloat timeAhead = (sfloat)(Time.ElapsedTime - m_RecordMostRecentFixedTime.MostRecentElapsedTime);
            sfloat timeStep = (sfloat)m_RecordMostRecentFixedTime.MostRecentDeltaTime;
            if (timeAhead <= sfloat.Zero || timeStep == sfloat.Zero)
                return;
            sfloat normalizedTimeAhead = math.clamp(timeAhead / timeStep, sfloat.Zero, sfloat.One);

            Dependency = JobHandle.CombineDependencies(Dependency, m_InputDependency);

            Dependency = new SmoothMotionJob
            {
                TranslationType = GetComponentTypeHandle<Translation>(true),
                RotationType = GetComponentTypeHandle<Rotation>(true),
                NonUniformScaleType = GetComponentTypeHandle<NonUniformScale>(true),
                ScaleType = GetComponentTypeHandle<Scale>(true),
                CompositeScaleType = GetComponentTypeHandle<CompositeScale>(true),
                PhysicsMassType = GetComponentTypeHandle<PhysicsMass>(true),
                InterpolationBufferType = GetComponentTypeHandle<PhysicsGraphicalInterpolationBuffer>(true),
                PhysicsGraphicalSmoothingType = GetComponentTypeHandle<PhysicsGraphicalSmoothing>(),
                LocalToWorldType = GetComponentTypeHandle<LocalToWorld>(),
                TimeAhead = timeAhead,
                NormalizedTimeAhead = normalizedTimeAhead
            }.ScheduleParallel(SmoothedDynamicBodiesGroup, 1, Dependency);

            // Combine implicit output dependency with user one
            m_OutputDependency = Dependency;

            // TODO: do we need to be able to inject this system's job handle as a dependency into the transform systems?

            // Invalidate input dependency since it's been used by now
            m_InputDependency = default;
        }

        [BurstCompile]
        struct SmoothMotionJob : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<Translation> TranslationType;
            [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
            [ReadOnly] public ComponentTypeHandle<NonUniformScale> NonUniformScaleType;
            [ReadOnly] public ComponentTypeHandle<Scale> ScaleType;
            [ReadOnly] public ComponentTypeHandle<CompositeScale> CompositeScaleType;
            [ReadOnly] public ComponentTypeHandle<PhysicsMass> PhysicsMassType;
            [ReadOnly] public ComponentTypeHandle<PhysicsGraphicalInterpolationBuffer> InterpolationBufferType;
            public ComponentTypeHandle<PhysicsGraphicalSmoothing> PhysicsGraphicalSmoothingType;
            public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
            public sfloat TimeAhead;
            public sfloat NormalizedTimeAhead;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                var hasNonUniformScale = batchInChunk.Has(NonUniformScaleType);
                var hasScale = batchInChunk.Has(ScaleType);
                var hasAnyScale = hasNonUniformScale || hasScale || batchInChunk.Has(CompositeScaleType);
                var hasPhysicsMass = batchInChunk.Has(PhysicsMassType);
                var hasInterpolationBuffer = batchInChunk.Has(InterpolationBufferType);

                NativeArray<Translation> positions = batchInChunk.GetNativeArray(TranslationType);
                NativeArray<Rotation> orientations = batchInChunk.GetNativeArray(RotationType);
                NativeArray<NonUniformScale> nonUniformScales = batchInChunk.GetNativeArray(NonUniformScaleType);
                NativeArray<Scale> scales = batchInChunk.GetNativeArray(ScaleType);
                NativeArray<CompositeScale> compositeScales = batchInChunk.GetNativeArray(CompositeScaleType);
                NativeArray<PhysicsMass> physicsMasses = batchInChunk.GetNativeArray(PhysicsMassType);
                NativeArray<PhysicsGraphicalSmoothing> physicsGraphicalSmoothings = batchInChunk.GetNativeArray(PhysicsGraphicalSmoothingType);
                NativeArray<PhysicsGraphicalInterpolationBuffer> interpolationBuffers = batchInChunk.GetNativeArray(InterpolationBufferType);
                NativeArray<LocalToWorld> localToWorlds = batchInChunk.GetNativeArray(LocalToWorldType);

                var defaultPhysicsMass = PhysicsMass.CreateKinematic(MassProperties.UnitSphere);
                for (int i = 0, count = batchInChunk.Count; i < count; ++i)
                {
                    var physicsMass = hasPhysicsMass ? physicsMasses[i] : defaultPhysicsMass;
                    var smoothing = physicsGraphicalSmoothings[i];
                    var currentVelocity = smoothing.CurrentVelocity;

                    var currentTransform = new RigidTransform(orientations[i].Value, positions[i].Value);
                    RigidTransform smoothedTransform;

                    // apply no smoothing (i.e., teleported bodies)
                    if (smoothing.ApplySmoothing == 0)
                    {
                        smoothedTransform = currentTransform;
                    }
                    else
                    {
                        if (hasInterpolationBuffer)
                        {
                            smoothedTransform = GraphicalSmoothingUtility.Interpolate(
                                interpolationBuffers[i].PreviousTransform, currentTransform, NormalizedTimeAhead);
                        }
                        else
                        {
                            smoothedTransform = GraphicalSmoothingUtility.Extrapolate(
                                currentTransform, currentVelocity, physicsMass, TimeAhead);
                        }
                    }

                    localToWorlds[i] = GraphicalSmoothingUtility.BuildLocalToWorld(
                        i, smoothedTransform,
                        hasAnyScale,
                        hasNonUniformScale, nonUniformScales,
                        hasScale, scales,
                        compositeScales
                    );

                    // reset smoothing to apply again next frame (i.e., finish teleportation)
                    smoothing.ApplySmoothing = 1;
                    physicsGraphicalSmoothings[i] = smoothing;
                }
            }
        }
    }
}
