using System;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace UnityS.Physics
{
    // INTERNAL UnityPhysics interface for jobs that iterate through the list of trigger events produced by the solver.
    // Important: Only use inside UnityPhysics code! Jobs in other projects should implement ITriggerEventsJob.
    [JobProducerType(typeof(ITriggerEventJobExtensions.TriggerEventJobProcess<>))]
    public interface ITriggerEventsJobBase
    {
        void Execute(TriggerEvent triggerEvent);
    }

#if !HAVOK_PHYSICS_EXISTS

    // Interface for jobs that iterate through the list of trigger events produced by the solver.
    public interface ITriggerEventsJob : ITriggerEventsJobBase
    {
    }

#endif

    public static class ITriggerEventJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default Schedule() implementation for ITriggerEventsJob.
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ITriggerEventsJobBase
        {
            // Should work only for UnityPhysics
            if (simulation.Type != SimulationType.UnityPhysics)
            {
                return inputDeps;
            }

            return ScheduleUnityPhysicsTriggerEventsJob(jobData, simulation, ref world, inputDeps);
        }
#else
        // In this case Schedule() implementation for ITriggerEventsJob is provided by the Havok.Physics assembly.
        // This is a stub to catch when that assembly is missing.
        //<todo.eoin.modifier Put in a link to documentation for this:
        [Obsolete("This error occurs when HAVOK_PHYSICS_EXISTS is defined but Havok.Physics is missing from your package's asmdef references. (DoNotRemove)", true)]
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps,
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF _causeCompileError = HAVOK_PHYSICS_MISSING_FROM_ASMDEF.HAVOK_PHYSICS_MISSING_FROM_ASMDEF)
            where T : struct, ITriggerEventsJobBase
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        // Schedules a trigger events job only for UnityPhysics simulation
        internal static unsafe JobHandle ScheduleUnityPhysicsTriggerEventsJob<T>(T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ITriggerEventsJobBase
        {
            SafetyChecks.CheckAreEqualAndThrow(SimulationType.UnityPhysics, simulation.Type);

            var data = new TriggerEventJobData<T>
            {
                UserJobData = jobData,
                EventReader = ((Simulation)simulation).TriggerEvents
            };

            // Ensure the input dependencies include the end-of-simulation job, so events will have been generated
            inputDeps = JobHandle.CombineDependencies(inputDeps, simulation.FinalSimulationJobHandle);
#if UNITY_2020_2_OR_NEWER
            var parameters = new JobsUtility.JobScheduleParameters(UnsafeUtility.AddressOf(ref data), TriggerEventJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Single);
#else
            var parameters = new JobsUtility.JobScheduleParameters(UnsafeUtility.AddressOf(ref data), TriggerEventJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
#endif
            return JobsUtility.Schedule(ref parameters);
        }

        internal unsafe struct TriggerEventJobData<T> where T : struct
        {
            public T UserJobData;
            [NativeDisableContainerSafetyRestriction] public TriggerEvents EventReader;
        }

        internal struct TriggerEventJobProcess<T> where T : struct, ITriggerEventsJobBase
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
#if UNITY_2020_2_OR_NEWER
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(TriggerEventJobData<T>), typeof(T), (ExecuteJobFunction)Execute);
#else
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(TriggerEventJobData<T>), typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
#endif
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref TriggerEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref TriggerEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                foreach (TriggerEvent triggerEvent in jobData.EventReader)
                {
                    jobData.UserJobData.Execute(triggerEvent);
                }
            }
        }
    }
}
