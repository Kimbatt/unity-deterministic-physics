using System;
using Unity.Entities;
using Unity.Jobs;
using UnityEngine.Assertions;

namespace UnityS.Physics.Systems
{
    // Simulates the physics world forwards in time
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(ExportPhysicsWorld)), AlwaysUpdateSystem]
    public class StepPhysicsWorld : SystemBase, IPhysicsSystem
    {
        private JobHandle m_InputDependency;
        private JobHandle m_OutputDependency;

        // The simulation implementation
        public ISimulation Simulation { get; private set; } = new DummySimulation();

        // The final simulation job handle produced by this system.
        // Systems which read the simulation results should depend on this.
        public JobHandle FinalSimulationJobHandle => Simulation.FinalSimulationJobHandle;

        // Optional callbacks to execute while scheduling the next simulation step
        private SimulationCallbacks m_Callbacks = new SimulationCallbacks();

        // Simulation factory
        public delegate ISimulation SimulationCreator();
        private readonly SimulationCreator[] m_SimulationCreators = new SimulationCreator[k_NumSimulationTypes];

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        ExportPhysicsWorld m_ExportPhysicsWorldSystem;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;

        // needs to match the number of SimulationType enum members
        internal static int k_NumSimulationTypes = 3;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_ExportPhysicsWorldSystem = World.GetOrCreateSystem<ExportPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();

#if !NET_DOTS
            Assert.AreEqual(Enum.GetValues(typeof(SimulationType)).Length, k_NumSimulationTypes);
#endif

            RegisterSimulation(SimulationType.NoPhysics, () => new DummySimulation());
            RegisterSimulation(SimulationType.UnityPhysics, () => new Simulation());
            RegisterSimulation(SimulationType.HavokPhysics, () =>
                throw new NotSupportedException("Havok Physics package not present. Use the package manager to add it."));

            base.OnCreate();
        }

        protected override void OnDestroy()
        {
            Simulation.Dispose();
            base.OnDestroy();
        }

        // Register a simulation creator
        public void RegisterSimulation(SimulationType type, SimulationCreator creator)
        {
            m_SimulationCreators[(int)type] = creator;
        }

        // Enqueue a callback to run during scheduling of the next simulation step
        public void EnqueueCallback(SimulationCallbacks.Phase phase, SimulationCallbacks.Callback callback, JobHandle dependency = default(JobHandle))
        {
            m_Callbacks.Enqueue(phase, callback, dependency);
        }

        protected override void OnUpdate()
        {
            // Combine implicit input dependency with the user one
            var handle = JobHandle.CombineDependencies(Dependency, m_InputDependency);

            PhysicsStep stepComponent = PhysicsStep.Default;
            if (HasSingleton<PhysicsStep>())
            {
                stepComponent = GetSingleton<PhysicsStep>();
            }

            // Swap the simulation implementation if the requested type changed
            if (Simulation.Type != stepComponent.SimulationType)
            {
                Simulation.Dispose();
                Simulation = m_SimulationCreators[(int)stepComponent.SimulationType]();
            }

            sfloat timeStep = (sfloat)Time.DeltaTime;

            // Schedule the simulation jobs
            Simulation.ScheduleStepJobs(new SimulationStepInput()
            {
                World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                TimeStep = timeStep,
                Gravity = stepComponent.Gravity,
                SynchronizeCollisionWorld = stepComponent.SynchronizeCollisionWorld > 0,
                NumSolverIterations = stepComponent.SolverIterationCount,
                SolverStabilizationHeuristicSettings = stepComponent.SolverStabilizationHeuristicSettings
            }, m_Callbacks, handle, stepComponent.MultiThreaded > 0);

            // Clear the callbacks. User must enqueue them again before the next step.
            m_Callbacks.Clear();

            // Return the final simulation handle
            // (Not FinalJobHandle since other systems shouldn't need to depend on the dispose jobs)
            m_OutputDependency = FinalSimulationJobHandle;

            // Combine implicit output dependency with user one
            Dependency = JobHandle.CombineDependencies(m_OutputDependency, Dependency);

            // Inform next systems in the pipeline of their dependency
            m_ExportPhysicsWorldSystem.AddInputDependency(m_OutputDependency);
            m_EndFramePhysicsSystem.AddInputDependency(Simulation.FinalJobHandle);

            // Invalidate input dependency since it's been used now
            m_InputDependency = default;
        }

        public void AddInputDependency(JobHandle inputDep)
        {
            m_InputDependency = JobHandle.CombineDependencies(m_InputDependency, inputDep);
        }

        public JobHandle GetOutputDependency()
        {
            return Simulation.FinalJobHandle;
        }

        // A simulation which does nothing
        class DummySimulation : ISimulation
        {
            public SimulationType Type => SimulationType.NoPhysics;

            public void Dispose() { }
            public void Step(SimulationStepInput input) { }
            [Obsolete("ScheduleStepJobs() has been deprecated. Please use the new method taking a bool as the last parameter. (RemovedAfter 2021-02-15)", true)]
            public SimulationJobHandles ScheduleStepJobs(SimulationStepInput input, SimulationCallbacks callbacks, JobHandle inputDeps, int threadCountHint = 0) =>
                new SimulationJobHandles(inputDeps);
            public SimulationJobHandles ScheduleStepJobs(SimulationStepInput input, SimulationCallbacks callbacks, JobHandle inputDeps, bool multiThreaded = true) =>
                new SimulationJobHandles(inputDeps);
            public JobHandle FinalSimulationJobHandle => new JobHandle();
            public JobHandle FinalJobHandle => new JobHandle();
        }
    }
}
