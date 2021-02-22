using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;
using UnityS.Physics.Systems;
using UnityS.Transforms;

namespace UnityS.Physics.GraphicsIntegration
{
    /// <summary>
    /// A system that writes to a rigid body's <see cref="PhysicsGraphicalInterpolationBuffer"/> component by copying its <c>Translation</c>, <c>Rotation</c>, and <see cref="PhysicsVelocity"/> before physics steps.
    /// These values are used for bodies whose graphics representations will be interpolated by the <see cref="SmoothRigidBodiesGraphicalMotion"/> system.
    /// Add a <c>WriteGroupAttribute</c> to your own component if you need to use different values (as with a character controller).
    ///
    /// NOTE: Consider the case when an interpolated rigid body needs to be teleported (i.e. have its <c>Translation</c>,
    /// <c>Rotation</c> or <see cref="PhysicsVelocity"/> components changed directly), specifically
    /// after this system is updated and before <see cref="SmoothRigidBodiesGraphicalMotion"/> is updated.
    /// In that case, you should set associated <see cref="PhysicsGraphicalSmoothing.ApplySmoothing"/> to 0.
    /// or assign the appropriate new <see cref="PhysicsGraphicalInterpolationBuffer"/> component values as well.
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(ExportPhysicsWorld))]
    public class BufferInterpolatedRigidBodiesMotion : SystemBase, IPhysicsSystem
    {
        JobHandle m_InputDependency;
        JobHandle m_OutputDependency;

        ExportPhysicsWorld m_ExportPhysicsWorldSystem;

        /// <summary>
        /// An entity query matching dynamic rigid bodies whose graphical motion should be interpolated.
        /// </summary>
        public EntityQuery InterpolatedDynamicBodiesGroup { get; private set; }

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
            m_ExportPhysicsWorldSystem = World.GetOrCreateSystem<ExportPhysicsWorld>();

            InterpolatedDynamicBodiesGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity),
                    typeof(Translation),
                    typeof(Rotation),
                    typeof(PhysicsGraphicalInterpolationBuffer)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsExclude)
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });

            RequireForUpdate(InterpolatedDynamicBodiesGroup);
        }

        protected override void OnUpdate()
        {
            // Combine implicit input dependency with the user one
            Dependency = JobHandle.CombineDependencies(Dependency, m_InputDependency);

            Dependency = new UpdateInterpolationBuffersJob
            {
                TranslationType = GetComponentTypeHandle<Translation>(true),
                RotationType = GetComponentTypeHandle<Rotation>(true),
                PhysicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true),
                InterpolationBufferType = GetComponentTypeHandle<PhysicsGraphicalInterpolationBuffer>()
            }.ScheduleParallel(InterpolatedDynamicBodiesGroup, 1, Dependency);

            // Combine implicit output dependency with user one
            m_OutputDependency = Dependency;

            // Inform next system in the pipeline of its dependency
            m_ExportPhysicsWorldSystem.AddInputDependency(m_OutputDependency);

            // Invalidate input dependency since it's been used by now
            m_InputDependency = default;
        }

        [BurstCompile]
        unsafe struct UpdateInterpolationBuffersJob : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            [ReadOnly] public ComponentTypeHandle<Translation> TranslationType;
            [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
            public ComponentTypeHandle<PhysicsGraphicalInterpolationBuffer> InterpolationBufferType;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                NativeArray<PhysicsVelocity> physicsVelocities = batchInChunk.GetNativeArray(PhysicsVelocityType);
                NativeArray<Translation> positions = batchInChunk.GetNativeArray(TranslationType);
                NativeArray<Rotation> orientations = batchInChunk.GetNativeArray(RotationType);
                NativeArray<PhysicsGraphicalInterpolationBuffer> interpolationBuffers = batchInChunk.GetNativeArray(InterpolationBufferType);

                var dst = interpolationBuffers.GetUnsafePtr();
                var count = batchInChunk.Count;

                var sizeBuffer = UnsafeUtility.SizeOf<PhysicsGraphicalInterpolationBuffer>();
                var sizeOrientation = UnsafeUtility.SizeOf<quaternion>();
                var sizePosition = UnsafeUtility.SizeOf<float3>();
                var sizeVelocity = UnsafeUtility.SizeOf<PhysicsVelocity>();

                UnsafeUtility.MemCpyStride(
                    dst, sizeBuffer,
                    orientations.GetUnsafeReadOnlyPtr(), sizeOrientation,
                    sizeOrientation,
                    count
                );
                UnsafeUtility.MemCpyStride(
                    (void*)((long)dst + sizeOrientation), sizeBuffer,
                    positions.GetUnsafeReadOnlyPtr(), sizePosition,
                    sizePosition,
                    count
                );
                UnsafeUtility.MemCpyStride(
                    (void*)((long)dst + sizeOrientation + sizePosition), sizeBuffer,
                    physicsVelocities.GetUnsafeReadOnlyPtr(), sizeVelocity,
                    sizeVelocity,
                    count
                );
            }
        }
    }
}
