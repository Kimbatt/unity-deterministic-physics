using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Physics.Systems;

namespace UnityS.Physics.GraphicsIntegration
{
    /// <summary>
    /// A system that writes to a body's <see cref="PhysicsGraphicalSmoothing"/> component by copying its <see cref="PhysicsVelocity"/> after physics has stepped.
    /// These values are used for bodies whose graphics representations will be smoothed by the <see cref="SmoothRigidBodiesGraphicalMotion"/> system.
    /// Add a <c>WriteGroupAttribute</c> to your own component if you need to use a different value (as with a character controller).
    /// </summary>
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(ExportPhysicsWorld))]
    public class CopyPhysicsVelocityToSmoothing : SystemBase, IPhysicsSystem
    {
        JobHandle m_InputDependency;
        JobHandle m_OutputDependency;

        ExportPhysicsWorld m_ExportPhysicsWorld;
        SmoothRigidBodiesGraphicalMotion m_SmoothRigidBodiesGraphicalMotion;

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

            m_ExportPhysicsWorld = World.GetOrCreateSystem<ExportPhysicsWorld>();
            m_SmoothRigidBodiesGraphicalMotion = World.GetOrCreateSystem<SmoothRigidBodiesGraphicalMotion>();

            SmoothedDynamicBodiesGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity),
                    typeof(PhysicsGraphicalSmoothing)
                },
                None = new ComponentType[]
                {
                    typeof(PhysicsExclude)
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });

            RequireForUpdate(SmoothedDynamicBodiesGroup);
        }

        protected override void OnUpdate()
        {
            Dependency = JobHandle.CombineDependencies(Dependency, m_ExportPhysicsWorld.GetOutputDependency());

            // Combine implicit input dependency with the user one
            Dependency = JobHandle.CombineDependencies(Dependency, m_InputDependency);

            Dependency = new CopyPhysicsVelocityJob
            {
                PhysicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true),
                PhysicsGraphicalSmoothingType = GetComponentTypeHandle<PhysicsGraphicalSmoothing>()
            }.ScheduleParallel(SmoothedDynamicBodiesGroup, 1, Dependency);

            // Combine implicit output dependency with user one
            m_OutputDependency = Dependency;

            // Inform next system in the pipeline of its dependency
            m_SmoothRigidBodiesGraphicalMotion.AddInputDependency(m_OutputDependency);

            // Invalidate input dependency since it's been used by now
            m_InputDependency = default;
        }

        [BurstCompile]
        unsafe struct CopyPhysicsVelocityJob : IJobEntityBatch
        {
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            public ComponentTypeHandle<PhysicsGraphicalSmoothing> PhysicsGraphicalSmoothingType;

            public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
            {
                NativeArray<PhysicsVelocity> physicsVelocities = batchInChunk.GetNativeArray(PhysicsVelocityType);
                NativeArray<PhysicsGraphicalSmoothing> physicsGraphicalSmoothings = batchInChunk.GetNativeArray(PhysicsGraphicalSmoothingType);

                UnsafeUtility.MemCpyStride(
                    physicsGraphicalSmoothings.GetUnsafePtr(), UnsafeUtility.SizeOf<PhysicsGraphicalSmoothing>(),
                    physicsVelocities.GetUnsafeReadOnlyPtr(), UnsafeUtility.SizeOf<PhysicsVelocity>(),
                    UnsafeUtility.SizeOf<PhysicsVelocity>(),
                    physicsVelocities.Length
                );
            }
        }
    }
}
