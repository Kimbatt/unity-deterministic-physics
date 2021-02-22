using System;
using Unity.Jobs;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    // Implementations of ISimulation
    public enum SimulationType
    {
        NoPhysics,                    // A dummy implementation which does nothing
        UnityPhysics,                 // Default C# implementation
        HavokPhysics                  // Havok implementation (using C++ plugin)
    }

    // Parameters for a simulation step
    public struct SimulationStepInput
    {
        public PhysicsWorld World; // Physics world to be stepped
        public sfloat TimeStep; // Portion of time to step the physics world for
        public float3 Gravity; // Gravity in the physics world
        public int NumSolverIterations; // Number of iterations to perform while solving constraints
        public bool SynchronizeCollisionWorld; // Whether to update the collision world after the step for more precise queries
        public Solver.StabilizationHeuristicSettings SolverStabilizationHeuristicSettings; // Settings for solver stabilization heuristic in Unity.Physics
    }

    // Result of ISimulation.ScheduleStepJobs()
    public struct SimulationJobHandles
    {
        public JobHandle FinalExecutionHandle;
        public JobHandle FinalDisposeHandle;

        public SimulationJobHandles(JobHandle handle)
        {
            FinalExecutionHandle = handle;
            FinalDisposeHandle = handle;
        }
    }

    // Interface for simulations
    public interface ISimulation : IDisposable
    {
        // The implementation type.
        SimulationType Type { get; }

        // Step the simulation.
        void Step(SimulationStepInput input);

        [Obsolete("ScheduleStepJobs() has been deprecated. Please use the new method taking a bool as the last parameter. (RemovedAfter 2021-02-15)", true)]
        SimulationJobHandles ScheduleStepJobs(SimulationStepInput input, SimulationCallbacks callbacks, JobHandle inputDeps, int threadCountHint);

        // Schedule a set of jobs to step the simulation.
        SimulationJobHandles ScheduleStepJobs(SimulationStepInput input, SimulationCallbacks callbacks, JobHandle inputDeps, bool multiThreaded = true);

        // The final scheduled simulation job.
        // Jobs which use the simulation results should depend on this.
        JobHandle FinalSimulationJobHandle { get; }

        // The final scheduled job, including all simulation and cleanup.
        // The end of each step should depend on this.
        JobHandle FinalJobHandle { get; }
    }
}
